#pragma once

#include <Windows.h>

#include "lin_alg.h"
#include <cmath>

struct vec2 {
	float x, y;
	vec2(float xx, float yy) : x(xx), y(yy) {}
	vec2() {};
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

__declspec(align(16))
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

	void *operator new(size_t size){
		void *p;
		ALIGNED_MALLOC16(p, size);
		return p;
	}

	void *operator new[](size_t size) {
		void *p;
		ALIGNED_MALLOC16(p, size);
		return p;
	}

	void operator delete(void *p) {
		ALIGNED_FREE(p);
	}

	void operator delete[](void *p) {
		ALIGNED_FREE(p);
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
	
	int split(float t, BEZIER4 *out) const;

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

struct BEZIER_PART {
	mat24 cpoints;
	mat24 matrix_repr;
	float tmin, tmax, padding1, padding2;
};

struct BEZIERWAVE {
	BEZIER_PART *parts;
	size_t num_parts;
	int add(const BEZIER4 &b);

};