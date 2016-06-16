#include "sound.h"

#include <Audioclient.h>
#include <audiopolicy.h>
#include <mmdeviceapi.h>
#include <ksmedia.h>
#include <stdio.h>
#include <cmath>
#include <limits>
#include <mutex>

#include <Avrt.h>

#include "wfedit.h"

// REFERENCE_TIME time units per second and per millisecond
#define REFTIMES_PER_SEC  10000000.0
#define REFTIMES_PER_MILLISEC  10000.0

#define IF_ERROR_EXIT(hr) do {\
		if (FAILED(hr)){\
			printf("sound.cpp:%d: WASAPI code error %lX\n", __LINE__, hr);\
			goto exit_err;\
		}\
} while(0)

#define SAFE_RELEASE(punk)  \
              if ((punk) != NULL)  \
                { (punk)->Release(); (punk) = NULL; }

const CLSID CLSID_MMDeviceEnumerator = __uuidof(MMDeviceEnumerator);
const IID IID_IMMDeviceEnumerator = __uuidof(IMMDeviceEnumerator);
const IID IID_IAudioClient = __uuidof(IAudioClient);
const IID IID_IAudioRenderClient = __uuidof(IAudioRenderClient);

#define SMPL_TYPE float

static wave_format_t wformat;
static UINT32 frame_size;
static int sound_system_initialized = 0;

static SMPL_TYPE *main_buffer = NULL;
static std::mutex main_buffer_lock;

UINT32 SND_get_frame_size() {
	return frame_size;
}

wave_format_t SND_get_format_info() {
	return wformat;
}

int SND_initialized() {
	return sound_system_initialized;
}

size_t SND_write_to_buffer(const float *data) {
	// data should contain 2*frame_size worth of floats normalized to [-1;1]
	//float max = (std::numeric_limits<short>::max)();
	float max = 1.0;
	
	main_buffer_lock.lock();

	for (int i = 0; i < frame_size; ++i) {
		SMPL_TYPE vl = (SMPL_TYPE)(max*data[2*i]);
		SMPL_TYPE vr = (SMPL_TYPE)(max*data[2*i + 1]);
		
		main_buffer[2*i] = vl;
		main_buffer[2*i + 1] = vr;
	}
	main_buffer_lock.unlock();

	return 1;
}


static int construct_wave_format_info(int samplerate, int nchannels, int bitdepth, WAVEFORMATEX *WFEX, wave_format_t *wft) {

	/*WFEX->wFormatTag = WAVE_FORMAT_PCM;
	WFEX->nChannels = nchannels;
	WFEX->nSamplesPerSec = samplerate;
	WFEX->nAvgBytesPerSec = samplerate * 2 * bitdepth / 8;
	WFEX->nBlockAlign = 2 * bitdepth / 8;
	WFEX->wBitsPerSample = bitdepth;
*/
	wft->num_channels = nchannels;
	wft->sample_rate = samplerate;
	wft->bit_depth = bitdepth;

	return 1;

}


#define TWO_PI (3.14159265359*2)

static inline float sin01(float alpha) {
	return 0.5*sin(alpha) + 0.5;
}

static inline float sin_minmax_Hz(float min, float max, float freq_Hz, float t) {
	return (max - min) / 2.0 * sin01(t * freq_Hz * TWO_PI);
}

static HRESULT find_smallest_128_aligned(IMMDevice *pDevice, IAudioClient *pAudioClient, const WAVEFORMATEX *wave_format) {

	REFERENCE_TIME DefaultDevicePeriod = 0, MinimumDevicePeriod = 0;
	HRESULT hr = pAudioClient->GetDevicePeriod(&DefaultDevicePeriod, &MinimumDevicePeriod);

	if (FAILED(hr)) return hr;

	printf("MinimumDevicePeriod: %lld, DefaultDevicePeriod: %lld\n", MinimumDevicePeriod, DefaultDevicePeriod);

	// TODO: explain this? :D
	//int n = ((int)(floorf(((float)MinimumDevicePeriod / REFTIMES_PER_SEC * wave_format->nSamplesPerSec) / 32.0) + 1)) * 32;
	int n = 512;
//	n *= 3;

	REFERENCE_TIME hnsPeriod = (REFERENCE_TIME)(REFTIMES_PER_SEC * (float)n / (float)wave_format->nSamplesPerSec + 0.5);

//	hr = pAudioClient->Initialize(AUDCLNT_SHAREMODE_EXCLUSIVE, AUDCLNT_STREAMFLAGS_EVENTCALLBACK, hnsPeriod, hnsPeriod, wave_format, NULL);
	hr = pAudioClient->Initialize(AUDCLNT_SHAREMODE_SHARED, NULL, hnsPeriod, NULL, wave_format, NULL);

	if (FAILED(hr)) {
		printf("IAudioClient::Initialize(): failed to set device period to minimum 128-byte-aligned value (%lld === %.4f ms), aborting.\n", hnsPeriod, (float)hnsPeriod / 10000.0);
		return hr;
	}
	else {
		printf("IAudioClient::Initialize(): success with n = %d (period = %lld === %.4f ms)\n", n, hnsPeriod, (float)hnsPeriod / 10000.0);
		return hr;
	}
	
}

HRESULT PlayAudioStream() {

	HRESULT hr;
	IMMDeviceEnumerator *pEnumerator = NULL;
	IMMDevice *pDevice = NULL;
	IAudioClient *pAudioClient = NULL;
	IAudioRenderClient *pRenderClient = NULL;
	BYTE *pData = NULL;
	DWORD flags = 0;
	HANDLE hEvent = NULL;
	HANDLE hTask = NULL;
	WAVEFORMATEX *pwfx;

	hr = CoInitialize(NULL);
	IF_ERROR_EXIT(hr);

	hr = CoCreateInstance(CLSID_MMDeviceEnumerator, NULL, CLSCTX_ALL, IID_IMMDeviceEnumerator, (void**)&pEnumerator);
	IF_ERROR_EXIT(hr);

	hr = pEnumerator->GetDefaultAudioEndpoint(eRender, eConsole, &pDevice);
	IF_ERROR_EXIT(hr);

	hr = pDevice->Activate(IID_IAudioClient, CLSCTX_ALL, NULL, (void**)&pAudioClient);
	IF_ERROR_EXIT(hr);


	//hr = pAudioClient->IsFormatSupported(AUDCLNT_SHAREMODE_EXCLUSIVE, &wave_format, NULL);
	hr = pAudioClient->GetMixFormat(&pwfx);
	WAVEFORMATEX *temp;
	IF_ERROR_EXIT(hr);

	construct_wave_format_info(44100, 2, 32, pwfx, &wformat);


	printf("sound: local mix format is %d-bit, %d-channel @ %d Hz\n", pwfx->wBitsPerSample, pwfx->nChannels, pwfx->nSamplesPerSec);

	hr = pAudioClient->IsFormatSupported(AUDCLNT_SHAREMODE_SHARED, pwfx, &temp);
	if (AUDCLNT_E_UNSUPPORTED_FORMAT == hr) {
		printf("WASAPI: default audio device does not support the requested WAVEFORMATEX (%d/%dch/%d bit)\n", pwfx->nSamplesPerSec, pwfx->nChannels, pwfx->wBitsPerSample);
		pAudioClient->Release();
		return hr;
	}

	IF_ERROR_EXIT(hr);
	hr = find_smallest_128_aligned(pDevice, pAudioClient, pwfx);
	IF_ERROR_EXIT(hr);
	
	hr = pAudioClient->GetBufferSize(&frame_size);
	IF_ERROR_EXIT(hr);

	INT32 frame_size_bytes = frame_size * pwfx->nChannels * pwfx->wBitsPerSample / 8;

	hr = pAudioClient->GetService(IID_IAudioRenderClient, (void**)&pRenderClient);
	IF_ERROR_EXIT(hr);

	//hEvent = CreateEvent(nullptr, false, false, nullptr);
	//if (hEvent == INVALID_HANDLE_VALUE) { printf("CreateEvent failed\n");  return -1; }
	
//	hr = pAudioClient->SetEventHandle(hEvent);
	//IF_ERROR_EXIT(hr);


	const size_t num_samples = frame_size_bytes / sizeof(SMPL_TYPE);

	main_buffer = new SMPL_TYPE[num_samples];
	memset(main_buffer, 0, sizeof(SMPL_TYPE) * num_samples);

	float min = (float)(std::numeric_limits<SMPL_TYPE>::min)();
	float max = (float)(std::numeric_limits<SMPL_TYPE>::max)();
	float halfmax = max / 2.0;
	float dt = 1.0 / (float)pwfx->nSamplesPerSec;

	float freq = 2*(float)pwfx->nSamplesPerSec / (float)frame_size;

	wformat.wave_freq = freq;
	wformat.cycle_duration_ms = 1.0 / freq * 1000.0;
	
//float freq = 440;

	//for (int i = 0; i < num_samples/2; ++i) {
	//	float t = (float)i * dt;
	//	main_buffer[2*i] = sin_minmax_Hz(min, max, freq, t); // L channel
	//	main_buffer[2*i + 1] = sin_minmax_Hz(min, max, freq, t); // R channel
	//}

	hr = pRenderClient->GetBuffer(frame_size, &pData);
	IF_ERROR_EXIT(hr);

	memcpy(pData, main_buffer, frame_size_bytes);

	hr = pRenderClient->ReleaseBuffer(frame_size, flags);
	IF_ERROR_EXIT(hr);

	printf("frame_size: %d, frame_size_bytes: %d\n", frame_size, frame_size_bytes);

	// increase thread priority for optimal av performance
	DWORD taskIndex = 0;
	hTask = AvSetMmThreadCharacteristics(TEXT("Pro Audio"), &taskIndex);
	if (hTask == NULL) {
		hr = E_FAIL;
		IF_ERROR_EXIT(hr);
	}

	double buffer_length_ms = (double)REFTIMES_PER_MILLISEC *
		(double)frame_size / (double)pwfx->nSamplesPerSec;

	sound_system_initialized = 1;

	hr = pAudioClient->Start();  // Start playing.
	IF_ERROR_EXIT(hr);

	while (wfedit_running()) {

		UINT32 padding;

		Sleep(buffer_length_ms / 2.0);

		hr = pAudioClient->GetCurrentPadding(&padding);
		IF_ERROR_EXIT(hr);

		hr = pRenderClient->GetBuffer(padding, &pData);
		IF_ERROR_EXIT(hr);

		size_t to_write = frame_size - padding;

		main_buffer_lock.lock();
		memcpy(pData, &main_buffer[2*padding], to_write*2*sizeof(float));
		main_buffer_lock.unlock();

		hr = pRenderClient->ReleaseBuffer(frame_size, 0);
		IF_ERROR_EXIT(hr);
	}


	hr = pAudioClient->Stop();  // Stop playing.
	IF_ERROR_EXIT(hr);

exit_err:

	SAFE_RELEASE(pEnumerator)
	SAFE_RELEASE(pDevice)
	SAFE_RELEASE(pAudioClient)
	SAFE_RELEASE(pRenderClient)

	if (hEvent != NULL) {
		CloseHandle(hEvent);
	}
	
	if (hTask != NULL) {
		AvRevertMmThreadCharacteristics(hTask);
	}
	
	printf("Exiting sound system...\n");

	return hr;
}
