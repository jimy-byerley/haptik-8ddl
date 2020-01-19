#include "model.h"
#include "linalg.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

using namespace la;

int main() {
	Delta delta;
	
	vec8 x(0.);
	x(2) = 15;
	
// 	for (int i=0; i<10; i++) {
// 		printf("run test %d\n", i);
		vec8 q;
		for (int i=0; i<N; i++) 	q(i) = float(random()) / RAND_MAX * M_PI_2;
		for (int i=0; i<N; i++)		printf("  %f", q(i));
		printf("\n");
		
// 		Delta::state s = delta.mgi(x);
		Delta::state s = delta.mgd_solve(q, x);
		x = s.X;
// 	}
	
	return 0;
}
