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
	la::mat8 mci(const la::vec8 X);	// J_cinematique = mci(X)
	la::vec8 mgi(const la::vec8 X);	// Q = mgi(X)
	la::vec8 mgd_solve(const la::vec8 Q, const la::vec8 X0); // calcule X pour Q par proximité a partir d'un point de départ
	
	Delta();	// construction des constantes pour accelerer les calculs
	
	la::mat3 RgA[N];	// matrices constantes pour les positionnement de A et B
	la::mat3 RgB[N];
};

/*
 * facilités internes
*/
float deg2rad(const float angle);
la::mat4 quat2mat(const la::vec4 rot, const la::vec3 pos);
la::mat4 rotz(const float angle);	// matrice de rotation autour de z

#endif
