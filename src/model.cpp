#include "model.h"
#include "linalg.h"
#include <math.h>

using namespace la;


Delta::Delta() {
	ra = 50;	// (mm) angle de placement des rotules sur la plateforme
	rb = 200;	// (mm) angle de placement des pivotes des moteurs
	R = 220; // (mm) longueur de tringle (tube noir)
	l = 80; // (mm) longueur de levier des servo
	const float phia_base = deg2rad(18);
	const float phib_base = deg2rad(6);
	float phia[N] = {
		phia_base,
		pi/4 - phia_base,
		pi/4 + phia_base,
		pi/2 - phia_base,
		pi/2 + phia_base,
		3*pi/4 - phia_base,
		3*pi/4 + phia_base,
		-phia_base
	};
	float phib[N] = {
		phib_base,
		pi/4 - phib_base,
		pi/4 + phib_base,
		pi/2 - phib_base,
		pi/2 + phib_base,
		3*pi/4 - phib_base,
		3*pi/4 + phib_base,
		-phib_base
	};
	for (size_t i=0; i<N; i++) {
		RgA[i] = rotz(phia[i]) * vec(ra, 0, 0, 1);
		b[i] = vec3(rotz(phib[i]) * vec(rb, 0, 0, 1));
	}
}

Delta::mgic Delta::mgi_complete(const vec8 &X) {
	mat4 bRe = quat2mat(vec2quat( *((vec3*) &X(3)) ));
	mat4 eRrg = quat2mat(vec2quat(vec(X(6), X(7), 0)));
	mat4 eRrd = quat2mat(vec2quat(vec(-X(6), -X(7), 0)));
	
	mat4 matg = bRe*eRrg;
	mat4 matd = bRe*eRrd;
	
	vec4 a[N];
	for (size_t i=0; i<4; i++) {
		a[i] = matg * RgA[i];
		a[i](3) = 1;
	}
	for (size_t i=4; i<8; i++) {
		a[i] = matd * RgA[i];
		a[i](3) = 1;
	}
	
	mgic results;
	vec8 &q = results.q;
	vec3 *c = results.c;
	
	// indices des a[i] a utiliser selon le plan
	size_t v1[] = {0, 3, 4, 7};
	size_t v2[] = {1, 2, 5, 6};
	
	for (size_t i=0; i<4; i++) {
		{ // intersections dans un plan x = a
			size_t i1 = v1[i];
			float x1,x2, z1,z2;
			
			vec3 S = a[i1];	// coord centre sphere
			vec3 K = vec(S(0), b[i1](1), S(2));	// coord projection de S sur le plan
			float sk = (S-K).norm();
			if (abs(sk) < R) {
				if ((l+R) >= (b[i1] - K).norm()) {
					// calcul solutions intersections de deux cercles dans le meme plan
					float L = sqrt(sq(R) - sq(sk));
					float A = b[i1](2) - K(2);
					float B = b[i1](0) - K(0);
					float a = 2*A;
					float b = 2*B;
					float c = sq(A) + sq(B) - sq(l) + sq(L);
					float delta = sq(2*a*c) - 4*(sq(a) + sqrt(b))*(sq(c) - sq(b)*sq(L));
					float z1 = (2*a*c - sqrt(delta)) / (2*(sq(a)+sq(b)) + K(2));
					// calcul x1 et x2
					if (b != 0) 	x2 = (c-a*(z2-K(2)))/b + K(0);
					else 			x2 = b/2 - sqrt(sq(l) - sq((2*c - sq(a))/(2*a)) ) + K(0); // erreur
				}
				// pas de solution
				else { /* TODO */ }
			}
			// 1 ou 0 solution
			else { /* TODO */ }
			
			c[i1] = vec(x2, b[i1](1), z2);
			q(i1) = atan(z2 / abs(x2-b[i1](0)));
		}
		
		{ // intersections dans un plan y = a
			size_t i2 = v2[i];
			float y1,y2, z1,z2;
			
			vec3 S = a[i2];	// coord centre sphere
			vec3 K = vec(b[i2](1), S(1), S(2));	// coord projection de S sur le plan
			float sk = (S-K).norm();
			if (abs(sk) < R) {
				if ((l+R) >= (b[i2] - K).norm()) {
					// calcul solutions intersections de deux cercles dans le meme plan
					float L = sqrt(sq(R) - sq(sk));
					float A = b[i2](2) - K(2);
					float B = b[i2](0) - K(0);
					float a = 2*A;
					float b = 2*B;
					float c = sq(A) + sq(B) - sq(l) + sq(L);
					float delta = sq(2*a*c) - 4*(sq(a) + sqrt(b))*(sq(c) - sq(b)*sq(L));
					float z1 = (2*a*c - sqrt(delta)) / (2*(sq(a)+sq(b)) + K(2));
					// calcul x1 et x2
					if (b != 0) 	y2 = (c-a*(z2-K(2)))/b + K(0);
					else			y2 = b/2 - sqrt(sq(l) - sq((2*c - sq(a))/(2*a)) ) + K(0);
				}
				// pas de solution
				else { /* TODO */ }
			}
			// 1 ou 0 solution
			else { /* TODO */ }
			
			c[i2] = vec(b[i2](1), y2, z2);
			q(i2) = atan(z2 / abs(y2-b[i2](1)));
		}
	}
	
	return results;
}



vec4 vec2quat(const vec3 &rot) {
	if (rot(0) == 0 && rot(1) == 0 && rot(2) == 0)
		return vec(1, 0, 0, 0);
	else {
		float angle = rot.norm();
		float s = sin(angle/2);
		float quat[] = {cos(angle/2),  s*rot(0)/angle, s*rot(1)/angle, s*rot(2)/angle};
		return vec4(quat);
	}
}
mat4 quat2mat(const vec4 &rot) {
	float q1 = rot(0);
	float q2 = rot(1);
	float q3 = rot(2);
	float q4 = rot(3);
	float m[] = {
		2*(q1*q1 + q2*q2)-1,	2*(q2*q3 - q1*q4), 		2*(q2*q4 + q1*q3), 	0,  
		2*(q2*q3 + q1*q4),		2*(q1*q1 + q3*q3)-1,	2*(q3*q4 - q1*q2), 	0,	 
		2*(q2*q4 - q1*q3),		2*(q3*q4 + q1*q2),		2*(q1*q2+q4*q4)-1,	0,
		0, 						0, 						0, 					1};
	return mat4(m);
}
mat4 rotz(const float angle) {
	float m[] = {
		cos(angle), -sin(angle), 0, 0, 
		sin(angle),  cos(angle), 0, 0,
		0,           0,          1, 0,
		0,			 0,          0, 1};
	return mat4(m);
}
