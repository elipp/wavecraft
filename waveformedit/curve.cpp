#include "curve.h"

#include <xmmintrin.h>
#include <smmintrin.h>

inline void print_mat4(const mat4 &M) {
	for (int i = 0; i < 4; ++i) {
		printf("(%.3f, %.3f, %.3f, %.3f)\n", M(0, i), M(1, i), M(2, i), M(3, i));
	}
	
	printf("\n");
}

static int split_bezier(float t, const mat24& points24, BEZIER4_fragment *out);

const mat4 BEZIER4::weights = mat4(
	vec4(1, -3, 3, -1), 
	vec4(0, 3, -6, 3), 
	vec4(0, 0, 3, -3), 
	vec4(0, 0, 0, 1));

const mat4 BEZIER4::weights_derivative = mat4(
	vec4(1, -2, 1, 0),
	vec4(0, 2, -2, 0),
	vec4(0, 0, 1, 0),
	vec4(0, 0, 0, 0));

const mat4 CATMULLROM4::weights = mat4(
	vec4(1, 0, -3, 2),
	vec4(0, 0, 3, -2),
	vec4(0, 1, -2, 1),
	vec4(0, 0, -1, 1));

static inline vec2 bezier2(const vec2 &a, const vec2 &b, float t) {
	return (1 - t)*a + t*b;
}

static inline vec2 bezier3(const vec2 &a, const vec2 &b, const vec2 &c, float t) {
	return bezier2(bezier2(a, b, t), bezier2(b, c, t), t);
}

static inline vec2 bezier4(const vec2 &a, const vec2 &b, const vec2 &c, const vec2 &d, float t) {
	return bezier2(bezier3(a, b, c, t), bezier3(b, c, d, t), t);
}

static inline mat24 multiply44_24(const mat4 &M44, const mat24 &M24) {
	return mat24(M44*M24.columns[0], M44*M24.columns[1]);
}

static inline vec2 multiply4_24(const vec4 &V4, const mat24 &M24) {
	return vec2(dot4(V4, M24.columns[0]), dot4(V4, M24.columns[1]));
}

vec2 operator*(float c, const vec2& v) {
	return vec2(c*v.x, c*v.y);
}

vec2 operator*(const vec2& v, float c) {
	return c * v;
}

vec2 BEZIER4::evaluate(float t) const {
	//return bezier4(control_points[0], control_points[1], control_points[2], control_points[3], t);
	float t2 = t*t;
	float t3 = t2*t;
	vec4 tv(1, t, t2, t3);

	return multiply4_24(tv, matrix_repr);
}

float BEZIER4::dydx(float t, float dt) const {

	t = t + dt > 1.0 ? t - dt : t;
	t = t - dt < 0.0 ? t + dt : t;

	vec2 p1 = evaluate(t + dt);
	vec2 p0 = evaluate(t);

	return (p1.y - p0.y) / (p1.x - p0.x);

}

float BEZIER4::dxdt(float t, float dt) const {
	t = t + dt > 1.0 ? t - dt : t;
	t = t - dt < 0.0 ? t + dt : t;
	
	vec2 p1 = evaluate(t + dt);
	vec2 p0 = evaluate(t);
	
	return (p1.x - p0.x) / dt;
}

float BEZIER4::dydt(float t, float dt) const {
	t = t + dt > 1.0 ? t - dt : t;
	t = t - dt < 0.0 ? t + dt : t;

	vec2 p1 = evaluate(t + dt);
	vec2 p0 = evaluate(t);

	return (p1.y - p0.y) / dt;
}

vec2 BEZIER4::evaluate_derivative(float t) const {
	float t2 = t*t;
	vec4 vt(1, t, t2, 0);
	vec2 d = multiply4_24(vt, derivative_mrepr);
	return d;
}


BEZIER4::BEZIER4(const vec2 &aP0, const vec2 &aP1, const vec2 &aP2, const vec2 &aP3) 
	: P0(aP0), P1(aP1), P2(aP2), P3(aP3) {

	points24 = mat24(P0, P1, P2, P3);
	matrix_repr = multiply44_24(BEZIER4::weights, points24);

	derivative_p24 = mat24(
		vec2(3 * (P1 - P0)),
		vec2(3 * (P2 - P1)),
		vec2(3 * (P3 - P2)),
		vec2(0, 0));

	derivative_mrepr = multiply44_24(BEZIER4::weights_derivative, derivative_p24);
};

BEZIER4::BEZIER4(const mat24 &PV) 
	: points24(PV) {

	P0 = points24.row(0);
	P1 = points24.row(1);
	P2 = points24.row(2);
	P3 = points24.row(3);

	matrix_repr = multiply44_24(BEZIER4::weights, points24);

	derivative_p24 = mat24(
		vec2(3 * (P1 - P0)),
		vec2(3 * (P2 - P1)),
		vec2(3 * (P3 - P2)),
		vec2(0, 0));

	derivative_mrepr = multiply44_24(BEZIER4::weights_derivative, derivative_p24);
};


CATMULLROM4 BEZIER4::convert_to_CATMULLROM4() const {
	return CATMULLROM4(
		P3 + 6 * (P0 - P1),
		P0,
		P3,
		P1 + 6 * (P3 - P2),
		1);
}

void BEZIER4::update() {
	points24 = mat24(P0, P1, P2, P3);
	matrix_repr = multiply44_24(BEZIER4::weights, points24);
}

static double avg_err = 0;
static int num_avg = 0;

static inline float LUT_find_y_for_x(float x, const vec2 *LUT, int buf_size, int *buff_offset) {
	int i = *buff_offset;

	int n = 0;

	for (; i < buf_size; ++i) {
		if (LUT[i].x >= x) {
			*buff_offset = i;
			break;
		}
		++n;
	}

	if (*buff_offset < i) {
		printf("(BEZIER4 helper)find_y_for_x: didn't find a suitable sample, using the last one.\n");
	}

	//printf("found point (%f, %f) in %d iterations for x target %f (fabs(DELTA) = %.2e)\n", LUT[i].x, LUT[i].y, n, x, fabs(LUT[i].x - x));

	//++num_avg;
	//avg_err += fabs(LUT[i].x - x);

	return LUT[i].y;
}

float *BEZIER4::sample_curve(UINT32 frame_size, int precision) const {
	float *samples = new float[frame_size * 2];

	size_t LUT_size = precision*frame_size;
	vec2 *LUT = new vec2[LUT_size];

	const float dt = 1.0 / (float)LUT_size;
	
	for (int i = 0; i < LUT_size; ++i) {
		float t = dt * (float)i;
		LUT[i] = evaluate(t);
	}

	const float dx = 1.0 / (float)frame_size;
	int buff_offset = 0;
	for (int i = 0; i < frame_size; ++i) {

		float target_x = i * dx;

		float y = LUT_find_y_for_x(target_x, LUT, LUT_size, &buff_offset);

		samples[2 * i] = y;
		samples[2 * i + 1] = y;

	}
	
	//printf("avg_err: %f\n", avg_err / (double)num_avg);

	delete[] LUT;
	
	return samples;
}

float *BEZIER4::sample_curve_noLUT(UINT32 frame_size, int precision) const {

	float *samples = new float[frame_size * 2];

	const float dx = 1.0 / (float)frame_size;
	const float dt = 1.0 / ((float)frame_size * precision);
	float t = 0;

	int buff_offset = 0;
	for (int i = 0; i < frame_size; ++i) {
		float target_x = i * dx;
		vec2 p = evaluate(t);

		while (p.x < target_x) {
			t += dt;
			p = evaluate(t);
		}

		samples[2 * i] = p.y;
		samples[2 * i + 1] = p.y;

	}

	return samples;
}



CATMULLROM4::CATMULLROM4(const vec2 &aP0, const vec2 &aP1, const vec2 &aP2, const vec2 &aP3, float a_tension)
: P0(aP0), P1(aP1), P2(aP2), P3(aP3), tension(a_tension) {
	const float &T = tension; // just an alias

	mat4 intermediate = 0.5*mat4(
		vec4(0, 0, -T, 0), 
		vec4(2, 0, 0, -T), 
		vec4(0, 2, T, 0), 
		vec4(0, 0, 0, T));
	
	points24 = mat24(P0, P1, P2, P3);

	matrix_repr = multiply44_24(CATMULLROM4::weights, multiply44_24(intermediate, points24));
}


CATMULLROM4::CATMULLROM4(const mat24 &PV, float a_tension)
	: points24(PV), tension(a_tension) {
	
	P0 = points24.row(0);
	P1 = points24.row(1);
	P2 = points24.row(2);
	P3 = points24.row(3);

	const float &T = tension; // just an alias

	mat4 intermediate = 0.5*mat4(
		vec4(0, 0, -T, 0),
		vec4(2, 0, 0, -T),
		vec4(0, 2, T, 0),
		vec4(0, 0, 0, T));

	matrix_repr = multiply44_24(CATMULLROM4::weights, multiply44_24(intermediate, points24));
}

vec2 CATMULLROM4::evaluate(float s) {
	float s2 = s*s;
	float s3 = s2*s;

	vec4 S(1, s, s2, s3);

	return multiply4_24(S, matrix_repr);
}


int CATMULLROM4::split(float s, CATMULLROM4 *out) const {

	//if (s < 0 || s > 1.0) return 0;

	//BEZIER4 B = this->convert_to_BEZIER4();
	//BEZIER4 SB[2];
	//B.split(s, &SB[0]);

	//// no idea if this is correct or not XD

	//out[0] = SB[0].convert_to_CATMULLROM4();
	//out[1] = SB[1].convert_to_CATMULLROM4();

	return 1;
}

BEZIER4 CATMULLROM4::convert_to_BEZIER4() const {
	float coef = 1.0 / (6.0*tension);
	return BEZIER4(
		P1,
		P1 + coef*(P2 - P0),
		P2 + coef*(P3 - P1),
		P2);

}


BEZIER4_fragment::BEZIER4_fragment(const mat24 &a_points24, float a_tmin, float a_tmax) 
: tmin(a_tmin), tmax(a_tmax) {
	points24 = a_points24;
	matrix_repr = multiply44_24(BEZIER4::weights, points24);
	tscale = 1.0 / (tmax - tmin);
}

void BEZIER4_fragment::update() {
	// update matrix_repr based on points24
	matrix_repr = multiply44_24(BEZIER4::weights, points24);
}


static int split_bezier(float t, const mat24 &points24, BEZIER4_fragment *out) {
	if (t < 0.0 || t > 1.0) {
		return 0;
	}

	float tm1 = t - 1;
	float tm2 = tm1*tm1;
	float tm3 = tm2*tm1;

	float t2 = t*t;
	float t3 = t2*t;

	// Refer to http://pomax.github.io/bezierinfo/, §10: Splitting curves using matrices.

	mat4 first(
		vec4(1, -tm1, tm2, -tm3),
		vec4(0, t, -2 * (tm1)*t, 3 * tm2*t),
		vec4(0, 0, t2, -3 * tm1*t2),
		vec4(0, 0, 0, t3)
		);


	mat4 tmp = first.transposed();

	// lulz, might be faster (also a lot clearer) just to directly set the second matrix XD

	mat4 second = mat4(
		tmp.column(3),
		vec4(_mm_shuffle_ps(tmp.column(2).getData(), tmp.column(2).getData(), _MM_SHUFFLE(2, 1, 0, 3))),
		vec4(_mm_shuffle_ps(tmp.column(1).getData(), tmp.column(1).getData(), _MM_SHUFFLE(1, 0, 3, 2))),
		vec4(_mm_shuffle_ps(tmp.column(0).getData(), tmp.column(0).getData(), _MM_SHUFFLE(0, 3, 2, 1)))
		)
		.transposed();

	mat24 C1 = multiply44_24(first, points24);
	mat24 C2 = multiply44_24(second, points24);

	// out[0] range: [0; t[,
	// out[1] range: [t; 1]

	out[0] = BEZIER4_fragment(C1, 0, nextafter(t, 0.0)); // the float right before t
	out[1] = BEZIER4_fragment(C2, t, 1.0);

	return 1;
}


int SEGMENTED_BEZIER4::split(float at_t) {
	// first we need to look up which one of the curves has this t.
	BEZIER4_fragment *frag = NULL;

	for (auto &c : parts) {
		if (c.tmin <= at_t && c.tmax > at_t) {
			frag = &c;
			break;
		}
	}

	if (!frag) {
		printf("SEGMENTED_BEZIER4::split: error: didn't find valid segment for t = %f\n", at_t);
		return 0;
	}

	BEZIER4_fragment split[2];

	// the t's will need to be scaled with tscale

	// tscale is 1.0 / (delta T)

	float t_local = (1.0 - (frag->tmax - at_t) * frag->tscale);

	split_bezier(t_local, frag->points24, &split[0]);

	// and now, the fragments' tmin and tmax will need to be scaled to fit the whole curve

	split[0].tmin = frag->tmin;
	split[0].tmax = nextafter(at_t, 0.0);
	split[0].tscale = 1.0 / (split[0].tmax - split[0].tmin);

	split[1].tmin = at_t;
	split[1].tmax = frag->tmax;
	split[1].tscale = 1.0 / (split[1].tmax - split[1].tmin);

	size_t offset = frag - &(*parts.begin());

	// then insert into curve

	// replace the previous fragment
	parts[offset] = split[0];
	matrix_reprs[offset] = split[0].matrix_repr;
	points_replace4(offset, split[0].points24);

	parts.insert(parts.begin() + (offset + 1), split[1]);
	matrix_reprs.insert(matrix_reprs.begin() + (offset + 1), split[1].matrix_repr);
	points_insert4(offset + 1, split[1].points24);

	printf("SEGMENTED_BEZIER::split: splitting curve at t = %f, which falls under the segment %lu.\nGot local_t = %f, \ns0 range: [%f, %f[ \ns1 range: [%f, %f], parts.size() = %d\n",
		at_t, offset, t_local, split[0].tmin, split[0].tmax, split[1].tmin, split[1].tmax, parts.size());

	return 1;
}

void SEGMENTED_BEZIER4::update_segment_index(int index) {
	auto &s = parts[index];
	matrix_reprs[index] = s.matrix_repr;
	points_replace4(index, s.points24); // this has the disadvantage of copying the 3 unchanged vec2s, but it looks neat
}

int SEGMENTED_BEZIER4::move_knot(int index, const vec2 &p) {
	if (index > parts.size()) {
		printf("SEGMENTED_BEZIER4::move_knot: error: requested index > num knots (%d > %d).\n", index, parts.size());
		return 0;
	}
	
	auto &s = parts[index >= parts.size() ? parts.size()-1 : index];

	if (index == parts.size()) {
		// this means very last control point value :P
		s.points24.assign_row(3, p);
		s.update();

		update_segment_index(parts.size() - 1);
		return 1;
	}

	s.points24.assign_row(0, p);
	s.update();

	update_segment_index(index);
			

	if (index > 0) {
		auto &sp = parts[index - 1];

		sp.points24.assign_row(3, p);
		sp.update();

		update_segment_index(index - 1);

	}

	return 1;
}

vec2 SEGMENTED_BEZIER4::get_knot(int index) const {
	if (index > parts.size()) {
		printf("SEGMENTED_BEZIER4::get_knot: error: requested index > num segments (%d > %d), returning last\n", index, parts.size());
		return parts.back().points24.row(3);
	}

	auto &s = parts[index];

	return s.points24.row(0);
}

vec2 SEGMENTED_BEZIER4::get_cp(int index) const {

	if (index > points.size()) {
		printf("SEGMENTED_BEZIER4::get_cp: error: requested index > num CPs (%d > %d), returning last\n", index, points.size());
		return points.back();
	}

	int mod = index % 4;
	int seg = index / 4;

	return parts[seg].points24.row(mod);
}


int SEGMENTED_BEZIER4::move_cp(int index, const vec2 &newp) {

	int mod = index % 4;
	int seg = index / 4;
	if (mod == 0 || mod == 3) {
		printf("SEGMENTED_BEZIER4::move_cp: attempted to call move_cp on a knot point (index %d), ignoring\n", index);
		return 0;
	}

	BEZIER4_fragment &p = parts[seg];
	p.points24.columns[0].assign(mod, newp.x);
	p.points24.columns[1].assign(mod, newp.y);

	p.matrix_repr = multiply44_24(BEZIER4::weights, p.points24);

	matrix_reprs[seg] = p.matrix_repr;
	points_replace4(seg, p.points24);

	return 1;
}

vec2 SEGMENTED_BEZIER4::evaluate(float t) const {

	const BEZIER4_fragment *frag = NULL;

	for (const auto &c : parts) {
		if (c.tmin <= t && c.tmax > t) {
			frag = &c;
			break;
		}
	}

	if (!frag) {
		printf("SEGMENTED_BEZIER4::evaluate: error: didn't find valid segment for t = %f\n", t);
		return vec2(0, 0);
	}

	float lt = 1 - (frag->tmax - t)*frag->tscale;

	float lt2 = lt*lt;
	float lt3 = lt2*lt;
	vec4 tv(1, lt, lt2, lt3);

	return multiply4_24(tv, frag->matrix_repr);

}



int SEGMENTED_BEZIER4::allocate_buffer(int num_channels, size_t framesize) {
	if (framesize != this->frame_size && this->samples == NULL) {
		this->frame_size = framesize;
		samples = new float[num_channels * framesize];
	}

	return 1;
}

int SEGMENTED_BEZIER4::update_buffer(int precision) {

	const float dx = 1.0 / (float)frame_size;
	const float dt = 1.0 / ((float)frame_size * precision);
	float t = 0;

	int buff_offset = 0;
	for (int i = 0; i < frame_size; ++i) {
		float target_x = i * dx;
		vec2 p = evaluate(t);

		while (p.x < target_x) {
			if (t > 1) { 
				printf("update_buffer: while p.x < %.4f: t > 1. Invalid curve.\n", target_x);
				return 0; 
			}
			t += dt;
			p = evaluate(t);
		}

		samples[2 * i] = p.y;
		samples[2 * i + 1] = p.y;

	}

	return 1;

}