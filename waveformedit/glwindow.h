#pragma once

#include <Windows.h>

#include "glad/glad.h"

#include <GLFW/glfw3.h>

#include "curve.h"

#define WIN_W 1600
#define WIN_H 900

GLFWwindow *create_GL_window(const char* title, int width, int height);
int init_GL();

void draw();

struct SEGMENTED_BEZIER4;

float *get_main_samples();
SEGMENTED_BEZIER4 get_main_bezier_copy();

int FFT_initialized();

void start_recording();
void stop_recording();

extern std::ofstream record;

