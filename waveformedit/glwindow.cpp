#include "glwindow.h"

#pragma comment(lib, "lin_alg.lib")
#pragma comment(lib, "glfw3.lib")
#pragma comment(lib, "libfftw3f-3.lib")

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
#include <sys/stat.h>

#include "wfedit.h"
//#include "texture.h"
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
static GLuint spectrum_VBOid, spectrum_VAOid;

static int recording = 0;

std::ofstream record;

bool fullscreen = false;
bool active = TRUE;

//static Texture *gradient_texture;

static ShaderProgram 
	*wave_shader,
	*point_shader,
	*grid_shader,
	*bezier_shader,
	*spectrum_shader;

static bool _main_loop_running = true;
bool main_loop_running() { return _main_loop_running; }
void stop_main_loop() { _main_loop_running = false; }

static SEGMENTED_BEZIER4 main_bezier;

SEGMENTED_BEZIER4 get_main_bezier_copy() { return main_bezier;  }

static mat4 projection, projection_inv;

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

void start_recording() {

	static const std::string default_base = "recording";

	std::string tn;
	struct stat fo;
	
	int r, s = 0;
	do {
		tn = default_base + std::to_string(s) + ".raw";
		r = stat(tn.c_str(), &fo);
		++s;
	} while (r == 0);
		
	printf("Recording raw data to file %s.\n", tn.c_str());
	record.open(tn, std::ios::binary);
}

void stop_recording() {

	printf("Recording stopped.\n");
	record.close();
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

float *get_main_samples() {
	return main_bezier.samples;
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
	
	main_bezier.update_buffer(32);
	SND_write_to_buffer(main_bezier.samples);

	glBindBuffer(GL_ARRAY_BUFFER, bezier_VBOid);
	glBufferSubData(GL_ARRAY_BUFFER, 0, main_bezier.matrix_reprs.size() * sizeof(mat24), &main_bezier.matrix_reprs[0]);

	glBindBuffer(GL_ARRAY_BUFFER, point_VBOid);
	glBufferSubData(GL_ARRAY_BUFFER, 0, main_bezier.points.size() * sizeof(vec2), &main_bezier.points[0]);

	if (FFT_initialized()) {
		float *spectrum = get_FFT_result();
		glBindBuffer(GL_ARRAY_BUFFER, spectrum_VBOid);
		glBufferSubData(GL_ARRAY_BUFFER, 0, get_FFT_size() * sizeof(float), spectrum);
	}

	//glUseProgram(wave_shader->getProgramHandle());
	//wave_shader->update_uniform_mat4("coefs_inv", m);
	//wave_shader->update_uniform_vec4("y_coords", vec4(y0, y1, y2, y3));

	//glBindBuffer(GL_ARRAY_BUFFER, wave_VBOid);
	//glBufferSubData(GL_ARRAY_BUFFER, 0, NUM_CURVES*sizeof(float), patch_buffer);

}


void draw() {

	update_data();
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(bezier_shader->getProgramHandle());
	glBindVertexArray(bezier_VAOid);
	bezier_shader->update_uniform_mat4("uMVP", projection);
	glDrawArrays(GL_PATCHES, 0, main_bezier.parts.size());

	glUseProgram(grid_shader->getProgramHandle());
	grid_shader->update_uniform_1f("tess_level", 5);
	grid_shader->update_uniform_mat4("uMVP", projection);
	glDrawArrays(GL_PATCHES, 0, 1);

	glUseProgram(point_shader->getProgramHandle());
	glBindVertexArray(point_VAOid);
	point_shader->update_uniform_mat4("uMVP", projection);
	glDrawArrays(GL_POINTS, 0, main_bezier.points.size());

	glUseProgram(spectrum_shader->getProgramHandle());
	glBindVertexArray(spectrum_VAOid);
	spectrum_shader->update_uniform_mat4("uMVP", projection);
	glDrawArrays(GL_POINTS, 0, get_FFT_size()/2);
	
	glBindVertexArray(0);

	/*grid_shader->update_uniform_1f("tess_level", 22);
	mvp = mvp * mat4::scale(0.50, 2.0, 0) * mat4::rotate(3.1415926536 / 2.0, 0, 0, 1) * mat4::translate(-0.5, 1.0, 0.0);
	grid_shader->update_uniform_mat4("uMVP", mvp);
	glDrawArrays(GL_PATCHES, 0, 1);*/

}

int init_GL() {

	glClearColor(0.0, 0.0, 0.0, 1.0);
	//glEnable(GL_DEPTH_TEST);

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

	std::unordered_map<GLuint, std::string> spectrum_attrib_bindings = {
		{0, "A_dB"}
	};

	//wave_shader = new ShaderProgram("shaders/wave", default_attrib_bindings);
	point_shader = new ShaderProgram("shaders/pointplot", default_attrib_bindings);
	grid_shader = new ShaderProgram("shaders/grid", default_attrib_bindings);
	bezier_shader = new ShaderProgram("shaders/bezier", bezier_attrib_bindings);
	spectrum_shader = new ShaderProgram("shaders/spectrum", spectrum_attrib_bindings);

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

	glGenVertexArrays(1, &spectrum_VAOid);
	glBindVertexArray(spectrum_VAOid);
	
	glEnableVertexAttribArray(0);
	glGenBuffers(1, &spectrum_VBOid);
	glBindBuffer(GL_ARRAY_BUFFER, spectrum_VBOid);
	glBufferData(GL_ARRAY_BUFFER, get_FFT_size()*sizeof(float), NULL, GL_DYNAMIC_DRAW);

	glVertexAttribPointer(0, 1, GL_FLOAT, GL_FALSE, 0, 0);

	glBindVertexArray(0);

	main_bezier = SEGMENTED_BEZIER4(BEZIER4(vec2(0.0, 0.0), vec2(0.33, -1.0), vec2(0.66, 1.0), vec2(1.0, 0.0)));

	main_bezier.split(0.15);
	//main_bezier.split(0.30);
	main_bezier.split(0.45);
	//main_bezier.split(0.60);
	main_bezier.split(0.75);
	//main_bezier.split(0.90);

	projection = mat4::proj_ortho(-0.1, 1.1, -1.5, 1.5, -1.0, 1.0);
	projection_inv = projection.inverted();

	update_data();

	return 1;

}

static void error_callback(int error, const char* description) {
	printf("GLFW error: %s\n", description);
}
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		wfedit_stop();
		glfwSetWindowShouldClose(window, GLFW_TRUE);
	}
	else if (key == GLFW_KEY_R && action == GLFW_PRESS) {
		if (recording) {
			stop_recording();
			recording = 0;
		}
		else {
			start_recording();
			recording = 1;
		}
	}
}

static int mouse_button_state[2] = { 0, 0 };
static int drag_index = -1;

static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
	
	if (drag_index < 0) { return; }

	int index = (drag_index+1) / 4;
	int modulo = drag_index % 4;

	vec4 p((2 * xpos) / WINDOW_WIDTH - 1, (1 - ((2 * ypos) / WINDOW_HEIGHT)), 0.0, 1.0);
	vec4 unproject = projection_inv * p;

	//printf("index = %d, unprojected = (%f, %f, %f, %f)\n", index, unproject(0), unproject(1), unproject(2), unproject(3));

	vec2 f(unproject(0), unproject(1));

	if (modulo == 0 || modulo == 3) {
		// then we're dealing with a knot
		if (drag_index == 0) {
			 main_bezier.move_knot(0, vec2(0.0, f.y));
		} else if (drag_index >= main_bezier.points.size() - 1) {
			main_bezier.move_knot(main_bezier.parts.size(), vec2(1.0, f.y));
		}
		else {
			main_bezier.move_knot(index, f);
		}
	}

	else {
		main_bezier.move_cp(drag_index, f);
	}

}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
	double x, y;
	glfwGetCursorPos(window, &x, &y);
	vec2 clickpos(x, y);

	int start_dragging = 0;

	if (mouse_button_state[button] == 1 && action == 0) {
		mouse_button_state[button] = 0;
		drag_index = -1;
	}
	else if (mouse_button_state[button] == 0 && action == 1) {
		mouse_button_state[button] = 1;
		start_dragging = 1;
	}

	printf("mouse button %d action at (%.1f, %.1f) (action %d)\n", button, x, y, action);

	int i = 0;
	for (auto &p : main_bezier.points) {
		vec4 pp = vec4(p.x, p.y, 0.0, 1.0);
		vec4 NDC = projection*pp;

		vec2 p((WINDOW_WIDTH/2.0) * (NDC(0) + 1.0), 
			WINDOW_HEIGHT - (WINDOW_HEIGHT/2.0)*(NDC(1) + 1.0));

		if ((p - clickpos).length() < 7.0 && start_dragging) {
			printf("point handle match, starting drag: %d (%.1f, %.1f)\n", i, p.x, p.y);
			drag_index = i;
			return;
		}
		++i;
	}

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

	glfwSetCursorPosCallback(window, cursor_position_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);

	return window;
}