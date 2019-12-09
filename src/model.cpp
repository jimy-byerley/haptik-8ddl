#include "model.h"
#include "linalh.g"
#include <math.h>

using namespace la;


Delta::Delta() {
	const float ra = 50;	// (mm)
	const float rb = 200;	// (mm)
	const float phia_base = deg2rad(18);
	const float phib_base = deg2rad(6);
	float phia[N] = {
		phia_base,
		M_PI/4 - phia_base,
		M_PI/4 + phia_base,
		M_PI/2 - phia_base,
		M_PI/2 + phia_base,
		3*M_PI/4 - phia_base,
		3*M_PI/4 + phia_base,
		-phia_base
	};
	float phib[N] = {
		phib_base,
		M_PI/4 - phib_base,
		M_PI/4 + phib_base,
		M_PI/2 - phib_base,
		M_PI/2 + phib_base,
		3*M_PI/4 - phib_base,
		3*M_PI/4 + phib_base,
		-phib_base
	};
	for (size_t i=0; i<N; i++) {
		RgA[i] = vec4(rotz(phia[i]) * vec3({ra, 0, 0}));
		RgA[i][3] = 1;
		b[i] = rotz(phib[i]) * vec3({rb, 0, 0});
	}
}

vec8 Delta::mci(const vec8 &X) {
	bRe = quat2mat(vec2quat( *((vec3) &(X[3])) ));
	eRrg = quat2mat(vec2quat(vec3({X[6], X[7], 0})));
	eRrd = quat2mat(vec2quat(vec3({-X[6], -X[7], 0})));
	
	mat4 matg = bRe*eRrg;
	mat4 matd = bRe*eRgd;
	
	vec4 a[N];
	for (size_t i=0; i<4; i++) {
		a[i] = matg * vec3({ra, 0, 0});
		a[i][3] = 1;
	}
	for (size_t i=4; i<8; i++) {
		a[i] = matd * vec3({ra, 0, 0});
		a[i][3] = 1;
	}
	
	// indices des a[i] a utiliser selon le plan
	size_t v1[] = {0, 3, 4, 7};
	size_t v2[] = {1, 2, 5, 6};
		
	// plan x = a
	for (size_t i=0; i<4; i++) {
		vec3 S = a[v1[i]];	// coord centre sphere
		vec3 K = vec3({S[0], b[v1[i]][1], S[2]});	// coord projection de S sur le plan
		float sk = (S-K).norm();
		if (abs(sk) < R) {
			if ((l+R) >= (b[v1[i]] - K).norm()) {
				// calcul solutions intersections de deux cercles dans le meme plan
				float L = hypot(R, sk);
				vec3 A = b[v1[i]][2] - K[2];
				vec3 B = b[v1[i]][0] - K[0];
				float a = 2*(A);
				float b = 2*(B);
				float c = sq(A) + sq(B) - sq(l) + sq(L);
				float delta = sq(2*a*c) - 4*(sq(a) + sqt(b))*(sq(c) - sq(b)*sq(L));
				float z1 = (2*a*c - sqrt(delta)) / (2*(sq(a)+sq(b)) + K[2]);
				// calcul x1 et x2
				// ...
			}
		}
	}
	
	// ...
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
	float q1 = rot[0];
	float q2 = rot[1];
	float q3 = rot[2];
	float q4 = rot[3];
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
