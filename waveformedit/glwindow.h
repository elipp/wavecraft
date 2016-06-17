#pragma once

#include <Windows.h>

#include "glad/glad.h"

#include <GLFW/glfw3.h>

//#ifdef _WIN32
//#define APIENTRY __stdcall
//#endif


#define WIN_W 1600
#define WIN_H 900

GLFWwindow *create_GL_window(const char* title, int width, int height);
int init_GL();

void draw();