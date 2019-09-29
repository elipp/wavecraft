#pragma once

#include <Windows.h>
#include <cmath>
#include <vector>

#include "glwindow.h"

#include "lin_alg.h"
#include "alignment_allocator.h"

struct vec2 {
	float x, y;
	vec2(float xx, float yy) : x(xx), y(yy) {}
	vec2() : x(0), y(0) {};
	vec2 operator+(const vec2 &v) const {
		return vec2(this->x + v.x, this->y + v.y);
	}

	vec2 operator-(const vec2 &v) const {
		return vec2(this->x - v.x, this->y - v.y);
	}

	float length() const {
		return sqrt(x*x + y*y);
	}
};

vec2 operator*(float c, const vec2& v);
vec2 operator*(const vec2& v, float c);

//__declspec(align(16))
struct mat24 { // 2 columns, 4 rows
	vec4 columns[2];
	
	mat24() {};
	mat24(const vec4 &C0, const vec4 &C1) : columns{ C0, C1 } {};
	mat24(const vec2 &R0, const vec2 &R1, const vec2 &R2, const vec2 &R3) {
		columns[0] = vec4(R0.x, R1.x, R2.x, R3.x);
		columns[1] = vec4(R0.y, R1.y, R2.y, R3.y);
	}

	vec2 row(int row) const {
		return vec2(columns[0](row), columns[1](row));
	}

	void assign_row(int row, const vec2 &v) {
		columns[0].assign(row, v.x);
		columns[1].assign(row, v.y);
	}

};

struct BEZIER4;
struct CATMULLROM4;

struct BEZIER4 {

	static const mat4 weights;
	static const mat4 weights_derivative;

	vec2 P0, P1, P2, P3;
	mat24 points24;
	mat24 matrix_repr;

	mat24 derivative_p24;
	mat24 derivative_mrepr;

	vec2 evaluate(float t) const;
	float dydx(float t, float dt = 0.001) const;
	float dxdt(float t, float dt = 0.001) const;
	float dydt(float t, float dt = 0.001) const;

	vec2 evaluate_derivative(float t) const;

	BEZIER4(const vec2 &aP0, const vec2 &aP1, const vec2 &aP2, const vec2 &aP3);
	BEZIER4(const mat24 &PV);

	void update(); // this updates the points24 and matrix_repr values according to the P0, ..., P3 values.

	BEZIER4() {}
	
	float *sample_curve(UINT32 frame_size, int precision = 8) const;
	float *sample_curve_noLUT(UINT32 frame_size, int precision = 32) const;

	CATMULLROM4 convert_to_CATMULLROM4() const;

};

struct CATMULLROM4 {

	static const mat4 weights;
	vec2 P0, P1, P2, P3;
	mat24 points24;
	float tension;
	mat24 matrix_repr;
	vec2 evaluate(float t);

	CATMULLROM4(const vec2 &aP0, const vec2 &aP1, const vec2 &aP2, const vec2 &aP3, float tension = 1.0);
	CATMULLROM4(const mat24 &PV, float tension = 1.0);
	CATMULLROM4() {}

	int split(float s, CATMULLROM4 *out) const;

	BEZIER4 convert_to_BEZIER4() const;

};

struct BEZIER4_fragment {
	mat24 points24;
	mat24 matrix_repr;
	float tmin, tmax, tscale, padding0;

	BEZIER4_fragment(const mat24 &a_points24, float a_tmin, float a_tmax);

	void update(); // update matrix_repr based on points24

	BEZIER4_fragment() {}

};

struct SEGMENTED_BEZIER4 {
	std::vector < BEZIER4_fragment, AlignmentAllocator<BEZIER4_fragment, 16>> parts;
	std::vector < mat24, AlignmentAllocator<mat24, 16>> matrix_reprs; // this is needed for shaders/bezier
	std::vector<vec2> points; // this is needed for shaders/pointplot

	float *samples;
	size_t frame_size;

	int split(float at_t); // this is the primary method for using this thing

	int move_knot(int index, const vec2 &new_position); 
	// knot = the first and last CP of every segment. Whether they need to be congruent will depend on how this is implemented.
	// "knot at index n" will refer to the first CP of the n-th segment, and 4th CP of the n-1:th segment (if it exists).

	int move_cp(int index, const vec2 &new_position);

	void points_push4(const mat24 &p) {
		points.push_back(p.row(0));
		points.push_back(p.row(1));
		points.push_back(p.row(2));
		points.push_back(p.row(3));
		printf("points_push4: points.size() = %d\n", (int)points.size());
	}

	void points_replace4(int index, const mat24 &p) {
		
		for (int i = 0; i < 4; ++i) {
		//	printf("replaced point %d (%.3f, %.3f) with point (%.3f, %.3f)\n", index * 4 + i, points[index * 4 + i].x, points[index * 4 +i].y, p.row(i).x, p.row(i).y);
			points[index * 4 + i] = p.row(i);
		}
		//printf("points_replace4: points.size() = %d\n", (int)points.size());
	}

	void points_insert4(int index, const mat24 &p) {

		vec2 rows[4] = { p.row(0), p.row(1), p.row(2), p.row(3) };
		// lol this is weird af, but insert doesn't include the element pointed to by the iterator "last". so effectively inserting 0, 1, 2, 3 but NOT 4
		points.insert(points.begin() + 4 * index, &rows[0], &rows[4]);
	
	}

	vec2 evaluate(float t) const; // evaluate the segmented curve at t = t (will need to look up which curve has that t value within its range)

	vec2 get_knot(int index) const; 
	vec2 get_cp(int index) const;

	SEGMENTED_BEZIER4(const BEZIER4 &master) {	
		BEZIER4_fragment f;
		f.matrix_repr = master.matrix_repr;
		f.points24 = master.points24;
		f.tmin = 0;
		f.tmax = 1;
		f.tscale = 1;
		parts.push_back(f);
		matrix_reprs.push_back(f.matrix_repr);
		points_push4(f.points24);
		samples = NULL;
		frame_size = 0;
	}

	SEGMENTED_BEZIER4() {}

	void update_segment_index(int index);

	int allocate_buffer(int num_channels, size_t framesize);
	int update_buffer(int precision = 32);

};
