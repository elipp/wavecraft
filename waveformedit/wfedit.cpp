#include "wfedit.h"
#include "glwindow.h"
#include "lin_alg.h"
#include "sound.h"
#include "curve.h"
#include "timer.h"

#include <cstdio>
#include <iostream>
#include <cassert>
#include <fstream>
#include <mutex>
#include <thread>

static int program_running = 1;
static float *sampling_result = NULL;
static float *FFT_result = NULL;
static fftwf_complex *output_buffer = NULL;

static std::condition_variable FFT_condition;
static std::mutex FFT_wait_mutex;
static bool FFT_ready = false;

static const int FFT_SIZE = 4096;

static HANDLE FFT_event;

static int sound_thread_proc() {
	return PlayAudioStream();
}

int wfedit_running() {
	return program_running;
}

void wfedit_stop() {
	program_running = 0;
}

float *get_FFT_result() { return FFT_result; }
int get_FFT_size() { return FFT_SIZE; }

static void resample_curve() {
	
	SEGMENTED_BEZIER4 main = get_main_bezier_copy();

	const float precision_modifier = 8.0;

	const float dx = 1.0 / FFT_SIZE;
	const float dt = dx / precision_modifier;
	float t = 0;

	for (int i = 0; i < FFT_SIZE; ++i) {
		float target_x = i * dx;
		vec2 p = main.evaluate(t);

		while (p.x < target_x) {
			if (t > 1) {
				printf("FFT: while p.x < %.4f: t > 1. Invalid curve.\n", target_x);
			}
			t += dt;
			p = main.evaluate(t);
		}

		sampling_result[i] = p.y;
	}

}

static int FFT_thread_proc() {

	// could just recalculate with desired resolution if in another thread :P

	while (!SND_initialized()) Sleep(250);

	size_t input_size = FFT_SIZE;
	size_t output_size = input_size / 2 + 1;

	if (!output_buffer) output_buffer = static_cast<fftwf_complex*>(fftwf_malloc(output_size*sizeof(fftwf_complex)));
	if (!sampling_result) sampling_result = new float[input_size];
	if (!FFT_result) {
		printf("FFT: output_size = %lu\n", output_size);
		FFT_result = new float[output_size - 1];
	}
	

	int flags = FFTW_ESTIMATE;
	fftwf_plan plan = fftwf_plan_dft_r2c_1d(input_size, sampling_result, output_buffer, flags);

	timer_t lock_timer;

	while (wfedit_running()) {

		{
			std::unique_lock<std::mutex> lock(FFT_wait_mutex);
			FFT_condition.wait(lock, [] { return FFT_ready; });
		}

		resample_curve();

		fftwf_execute(plan);

		float input_size_recip = 1.0 / (float)input_size;

		for (int i = 1; i < output_size; ++i) {
			float mag = 2 * sqrt(output_buffer[i][0] * output_buffer[i][0] + output_buffer[i][1] * output_buffer[i][1]) * input_size_recip;
			float a = 20 * log10f(mag);
			//float freq = (i * (48000 / 2)) / (input_size / 2);
			FFT_result[i - 1] = a;
		}

		FFT_ready = false;
	}
	
	fftwf_free(output_buffer);
	fftwf_destroy_plan(plan);

	return 1;
}


int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {

#define ENABLE_CONSOLE

#ifdef ENABLE_CONSOLE
	if (AllocConsole()) {
		// for debugging those early-program fatal erreurz. this will screw up our framerate though.
		FILE *dummy;
		freopen_s(&dummy, "CONOUT$", "wt", stdout);

		SetConsoleTitle("debug output");
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_RED);
	}
#endif


	GLFWwindow *window = NULL;

	long wait = 0;
	static double time_per_frame_ms = 0;
		
	//DWORD sound_threadID;
	//CreateThread(NULL, 0, sound_thread_proc, NULL, 0, &sound_threadID);

	std::thread sound_thread(sound_thread_proc);

	window = create_GL_window("WFEDIT", WIN_W, WIN_H);

	if (!window) {
		return EXIT_FAILURE;
	}

	if (!init_GL()) {
		return EXIT_FAILURE;
	}
	
	std::thread FFT_thread(FFT_thread_proc);

	while (wfedit_running() && !glfwWindowShouldClose(window)) {

		draw();
		
		{
			std::unique_lock<std::mutex> lock(FFT_wait_mutex);
			FFT_ready = true;
			FFT_condition.notify_one();
		}

		glfwSwapBuffers(window);
		glfwPollEvents();

	}

	wfedit_stop(); // this is needed to stop the worker threadz

	// not sure if this is a good way to do this :D

	{
		std::unique_lock<std::mutex> lock(FFT_wait_mutex);
		FFT_ready = true;
		FFT_condition.notify_one();
	}


	FFT_thread.join();
	sound_thread.join();
	
	glfwDestroyWindow(window);
	glfwTerminate();

	exit(EXIT_SUCCESS);
}
