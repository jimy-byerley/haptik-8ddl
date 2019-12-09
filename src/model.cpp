#include "model.h"
#include "linalh.g"
#include <math.h>

using namespace la;


Delta::Delta() {
	const float ra = 50;	// (mm)
	const float rb = 200;	// (mm)
	Ta = mat4({1,0,0,ra,   0,1,0,0,  0,0,1,0,   0,0,0,1});
	Tb = mat4({1,0,0,rb,   0,1,0,0,  0,0,1,0,   0,0,0,1});
	for (size_t i=0; i<N; i++) {
		RgA[i] = rotz(phia[i]) * Ta;
		RgB[i] = rotz(phib[i]) * Tb;
	}
}

vec8 Delta::mci(const vec8 &X) {
	bRe = quat2mat(
}


float deg2rad(const float angle) {
	return angle * M_PI/180;
}
vec4 vec2quat(const vec3 &rot) {
	if (rot[0] == 0 && rot[1] == 0 && rot[2] == 0)
		return vec4({1, 0, 0, 0});
	else {
		float angle = rot.norm();
		float s = sin(angle/2);
		return vec4({cos(angle/2),  s*rot[0]/angle, s*rot[1]/angle, s*rot[2]/angle});
	}
}
mat4 quat2mat(const vec4 &rot, const vec3 &pos) {
	return mat3({
		2*(q1*q1 + q2*q2)-1,	2*(q2*q3 - q1*q4), 		2*(q2*q4 + q1*q3), 	pos[0],  
		2*(q2*q3 + q1*q4),		2*(q1*q1 + q3*q3)-1,	2*(q3*q4 - q1*q2), 	pos[1],	 
		2*(q2*q4 - q1*q3),		2*(q3*q4 + q1*q2),		2*(q1*q2+q4*q4)-1,	pos[2],
		0 						0 						0 					1});
}
mat4 rotz(const float angle) {
	return mat4({
		cos(angle), -sin(angle), 0, 0, 
		sin(angle),  cos(angle), 0, 0,
		0,           0,          1, 0,
		0,			 0,          0, 1});
}
