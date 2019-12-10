#ifndef _MODEL_H
#define _MODEL_H

#include "linalg.h"

static const size_t N = 8;
typedef la::Vector<float, N> vec8;
typedef la::Matrix<float, N, N> mat8;

/** 
 * 	structure contenant les constantes de calcul
*/
struct Delta {
	/// fonctions mises a disposition
	mat8 mci(const vec8 &X);	// J_cinematique = mci(X)
	vec8 mgi(const vec8 &X);	// Q = mgi(X)
	vec8 mgd_solve(const vec8 &Q, const vec8 &X0); // calcule X pour Q par proximité a partir d'un point de départ
	
	Delta();	// construction des constantes pour accelerer les calculs
	
	la::vec4 RgA[N];	// matrices constantes pour les positionnement de A et B
	la::vec3 b[N];
	float ra;
	float rb;
	float R;
	float l;
};

/*
 * facilités internes
*/

const float pi = M_PI;
inline float sq(const float x)		{ return x*x; }
inline float deg2rad(const float angle) { return angle * M_PI/180; }
la::vec4 vec2quat(const la::vec3 &rot);	// quaternion associé a la rotation autour du vecteur, et d'angle sa norme
la::mat4 quat2mat(const la::vec4 &rot);	// matrice de rotation associée au quaternion
la::mat4 rotz(const float angle);	// matrice de rotation autour de z

#endif
