#include "glwindow.h"

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>
#include <random>
#include <stdarg.h>

#include <cassert>
#include <signal.h>
#include <mutex>

#include "texture.h"
#include "shader.h"
#include "lin_alg.h"
#include "sound.h"
#include "curve.h"
#include "timer.h"

bool mouse_locked = false;

static HGLRC hRC = NULL;
static HDC hDC = NULL;
static HWND hWnd = NULL;
static HINSTANCE hInstance;

extern HINSTANCE win_hInstance;

static HWND hWnd_child = NULL;

unsigned WINDOW_WIDTH = WIN_W;
unsigned WINDOW_HEIGHT = WIN_H;

static GLuint wave_VBOid, wave_VAOid;
static GLuint bezier_VBOid, bezier_VAOid;
static GLuint point_VBOid, point_VAOid;

bool fullscreen = false;
bool active = TRUE;

static Texture *gradient_texture;
static ShaderProgram *wave_shader, *point_shader, *grid_shader, *bezier_shader;

static bool _main_loop_running = true;
bool main_loop_running() { return _main_loop_running; }
void stop_main_loop() { _main_loop_running = false; }

static SEGMENTED_BEZIER4 main_bezier;

#define NUM_CURVES 1

void kill_GL_window();

void set_cursor_relative_pos(int x, int y) {
	POINT pt;
	pt.x = x;
	pt.y = y;
	ClientToScreen(hWnd, &pt);
	SetCursorPos(pt.x, pt.y);
}

static std::string *convertLF_to_CRLF(const char *buf);


vec4 solve_equation_coefs(const float *points) {

	const float& a = points[0];
	const float& b = points[2];
	const float& c = points[4];
	const float& d = points[6];

	mat4 m = mat4(vec4(a*a*a, a*a, a, 1), vec4(b*b*b, b*b, b, 1), vec4(c*c*c, c*c, c, 1), vec4(d*d*d, d*d, d, 1));
	m.invert();

	vec4 correct = m.transposed() * vec4(points[1], points[3], points[5], points[7]);

	return correct;
}

static float *sample_buffer = NULL;

static float GT = 0;

static int allocate_static_buf(size_t size) {
	if (sample_buffer == NULL) {
		sample_buffer = new float[size];
		return 1;
	}
	return 0;
}



static int get_samples(const vec4 &coefs, wave_format_t *fmt) {
	UINT32 frame_size = SND_get_frame_size();

	float dt = 1.0 / (float)frame_size;
	float x = 0;

	for (int i = 0; i < frame_size; ++i) {
		float x2 = x*x;
		float x3 = x2*x;
		vec4 tmp = vec4(x3, x2, x, 1);
		
		float v = 0.6*dot4(coefs, tmp);

		//printf("v: %.4f\n", v);
	
		sample_buffer[2*i] = v;
		sample_buffer[2*i + 1] = v;

		x += dt;
	}


	return 1;

}

void update_data() {

	GT += 0.005;

	while (!SND_initialized()) { Sleep(250); }

	wave_format_t fmt = SND_get_format_info();
	//allocate_static_buf(fmt.num_channels * SND_get_frame_size());

	main_bezier.allocate_buffer(fmt.num_channels, SND_get_frame_size());
	// this won't do anything if it's already allocated

	//main_bezier.move_knot(4, vec2(main_bezier.get_knot(4).x, 1.1*sin(GT)));
	//main_bezier.move_knot(1, vec2(main_bezier.get_knot(1).x, sin(1.3*GT)));

	main_bezier.move_cp(6, vec2(main_bezier.get_cp(6).x, sin(GT)));
	main_bezier.move_cp(10, vec2(main_bezier.get_cp(10).x, 1.3*sin(5*GT)));
	
	main_bezier.update_buffer(32);
	SND_write_to_buffer(main_bezier.samples);

	glBindBuffer(GL_ARRAY_BUFFER, bezier_VBOid);
	glBufferSubData(GL_ARRAY_BUFFER, 0, main_bezier.matrix_reprs.size() * sizeof(mat24), &main_bezier.matrix_reprs[0]);

	glBindBuffer(GL_ARRAY_BUFFER, point_VBOid);
	glBufferSubData(GL_ARRAY_BUFFER, 0, main_bezier.points.size() * sizeof(vec2), &main_bezier.points[0]);


	static int k = 1;

	if (k) {
		for (int i = 0; i < main_bezier.points.size()/4; ++i) {
			for (int j = 0; j < 4; ++j) {
				printf("points[%d]: (%.3f, %.3f)\n", 4*i + j, main_bezier.points[4*i + j].x, main_bezier.points[4*i+j].y);
				printf("from parts: (%.3f, %.3f)\n", main_bezier.parts[i].points24.row(j).x, main_bezier.parts[i].points24.row(j).y);
			}

		}

	}
	k = 0;

	//glUseProgram(wave_shader->getProgramHandle());
	//wave_shader->update_uniform_mat4("coefs_inv", m);
	//wave_shader->update_uniform_vec4("y_coords", vec4(y0, y1, y2, y3));

	//glBindBuffer(GL_ARRAY_BUFFER, wave_VBOid);
	//glBufferSubData(GL_ARRAY_BUFFER, 0, NUM_CURVES*sizeof(float), patch_buffer);

}


void draw() {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	mat4 mvp = mat4::proj_ortho(-0.1, 1.1, -1.5, 1.5, -1.0, 1.0);

	update_data();

	glUseProgram(bezier_shader->getProgramHandle());
	glBindVertexArray(bezier_VAOid);
	bezier_shader->update_uniform_mat4("uMVP", mvp);
	glDrawArrays(GL_PATCHES, 0, main_bezier.parts.size());

	glUseProgram(grid_shader->getProgramHandle());
	grid_shader->update_uniform_1f("tess_level", 5);
	grid_shader->update_uniform_mat4("uMVP", mvp);
	glDrawArrays(GL_PATCHES, 0, 1);

	glUseProgram(point_shader->getProgramHandle());
	glBindVertexArray(point_VAOid);
	point_shader->update_uniform_mat4("uMVP", mvp);
	glDrawArrays(GL_POINTS, 0, main_bezier.points.size());
	
	glBindVertexArray(0);

	/*grid_shader->update_uniform_1f("tess_level", 22);
	mvp = mvp * mat4::scale(0.50, 2.0, 0) * mat4::rotate(3.1415926536 / 2.0, 0, 0, 1) * mat4::translate(-0.5, 1.0, 0.0);
	grid_shader->update_uniform_mat4("uMVP", mvp);
	glDrawArrays(GL_PATCHES, 0, 1);*/

}

int init_GL() {

	glClearColor(0.0, 0.0, 0.0, 1.0);
	glEnable(GL_DEPTH_TEST);

	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glEnable(GL_BLEND);

	glViewport(0, 0, WIN_W, WIN_H);

	const char* version = (const char*)glGetString(GL_VERSION);

	printf("OpenGL version information:\n%s\n\n", version);

	GLint max_elements_vertices, max_elements_indices;
	glGetIntegerv(GL_MAX_ELEMENTS_VERTICES, &max_elements_vertices);
	glGetIntegerv(GL_MAX_ELEMENTS_INDICES, &max_elements_indices);

	glPatchParameteri(GL_PATCH_VERTICES, 1);

	printf("GL_MAX_ELEMENTS_VERTICES = %d\nGL_MAX_ELEMENTS_INDICES = %d\n", max_elements_vertices, max_elements_indices);

#define ADD_ATTRIB(map, attrib_id, attrib_name) do {\
	(map).insert(std::make_pair<GLuint, std::string>((attrib_id), std::string(attrib_name)));\
	} while(0)

	std::unordered_map<GLuint, std::string> default_attrib_bindings = {
		{0, "pos_vs_in" }
	};

	std::unordered_map<GLuint, std::string> bezier_attrib_bindings = {
		{0, "COORD_COLUMN_X"}, {1, "COORD_COLUMN_Y"}
	};

	//wave_shader = new ShaderProgram("shaders/wave", default_attrib_bindings);
	point_shader = new ShaderProgram("shaders/pointplot", default_attrib_bindings);
	grid_shader = new ShaderProgram("shaders/grid", default_attrib_bindings);
	bezier_shader = new ShaderProgram("shaders/bezier", bezier_attrib_bindings);

	//glGenVertexArrays(1, &wave_VAOid);
	//glBindVertexArray(wave_VAOid);

	//glEnableVertexAttribArray(ATTRIB_POSITION);

	//glGenBuffers(1, &wave_VBOid);
	//glBindBuffer(GL_ARRAY_BUFFER, wave_VBOid);
	//glBufferData(GL_ARRAY_BUFFER, NUM_CURVES*sizeof(float), NULL, GL_DYNAMIC_DRAW);

	//glVertexAttribPointer(ATTRIB_POSITION, 1, GL_FLOAT, GL_FALSE, 1*sizeof(float), 0);

	//glBindVertexArray(0);

	glGenVertexArrays(1, &bezier_VAOid);
	glBindVertexArray(bezier_VAOid);

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

	glGenBuffers(1, &bezier_VBOid);
	glBindBuffer(GL_ARRAY_BUFFER, bezier_VBOid);
	glBufferData(GL_ARRAY_BUFFER, 64 * sizeof(mat24), NULL, GL_DYNAMIC_DRAW);

	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(mat24), 0);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(mat24), (LPVOID)(sizeof(mat24::columns[0])));

	glBindVertexArray(0);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	glGenVertexArrays(1, &point_VAOid);
	glBindVertexArray(point_VAOid);

	glEnableVertexAttribArray(0);

	glGenBuffers(1, &point_VBOid);
	glBindBuffer(GL_ARRAY_BUFFER, point_VBOid);
	glBufferData(GL_ARRAY_BUFFER, 4*64 * sizeof(vec2), NULL, GL_DYNAMIC_DRAW);

	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBindVertexArray(0);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	main_bezier = SEGMENTED_BEZIER4(BEZIER4(vec2(0.0, 0.0), vec2(0.33, -1.0), vec2(0.66, 1.0), vec2(1.0, 0.0)));

	main_bezier.split(0.15);
	main_bezier.split(0.35);
	main_bezier.split(0.70);
	main_bezier.split(0.60);

	update_data();

	return 1;

}

static void error_callback(int error, const char* description)
{
	printf("GLFW error: %s\n", description);
}
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GLFW_TRUE);
}

GLFWwindow *create_GL_window(const char* title, int width, int height) {
	GLFWwindow* window;

	glfwSetErrorCallback(error_callback);

	if (!glfwInit())
		return 0;

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	window = glfwCreateWindow(width, height, title, NULL, NULL);

	if (!window) {
		glfwTerminate();
		return 0;
	}

	glfwSetKeyCallback(window, key_callback);

	glfwMakeContextCurrent(window);
	gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
	glfwSwapInterval(1);

	return window;
}