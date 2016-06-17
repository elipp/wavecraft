#include "wfedit.h"
#include "glwindow.h"
#include "lin_alg.h"
#include "sound.h"
#include "curve.h"
#include "timer.h"

#include <cstdio>
#include <iostream>
#include <cassert>

static int program_running = 1;

static DWORD WINAPI sound_thread_proc(LPVOID lpParam) {
	return (DWORD)PlayAudioStream();
}

int wfedit_running() {
	return program_running;
}

static void wfedit_stop() {
	program_running = 0;
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
		
	DWORD sound_threadID;
	CreateThread(NULL, 0, sound_thread_proc, NULL, 0, &sound_threadID);

	window = create_GL_window("WFEDIT", WIN_W, WIN_H);
	if (!window) {
		return EXIT_FAILURE;
	}

	if (!init_GL()) {
		return EXIT_FAILURE;
	}

	bool running = true;

	while (!glfwWindowShouldClose(window))
	{
		draw();

		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	glfwDestroyWindow(window);
	glfwTerminate();

	exit(EXIT_SUCCESS);
}
