#include <stdio.h>
#include "linalg.h"

using namespace la;

typedef Matrix<float, 8, 8> mat8;
typedef Vector<float, 8> vec8;

int main() {
	vec3 v = pack(0, 2, 4);
	mat8 m1(2.);
	mat8 m2 = mat8::identity();
	
	printf("size of vector and an array: %i  %i\n", sizeof(vec8), sizeof(float)*8);
	printf("sizeof matrix and an array: %i   %i\n", sizeof(mat8), sizeof(float)*8*8);
	
	mat8 m3 = m1 * m2 + 2*mat8::identity();
	
	mat8 result = m3 * m3.inverse();
	
	for (int i=0; i<result.nrows(); i++) {
		for (int j=0; j<result.ncols(); j++) {
			printf("\t%f", result(i,j));
		}
		putchar('\n');
	}
	
	return 0;
}
