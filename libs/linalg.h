#ifndef _LINALG_H_
#define _LINALG_H_

#include <stddef.h>
#include <math.h>

namespace la {

	

template<class S, size_t dim>
struct Vector {
	S storage[dim];
	
public:
	// constructeurs
	Vector() {}
	Vector(const S v) {
		for (size_t i=0; i<dim; i++)	storage[i] = v;
	}
	Vector(const S * data) {
		for (size_t i=0; i<dim; i++)	storage[i] = data[i];
	}
	
	// methodes d'acces
	size_t ndim() const { return dim; }
	S * ptr() { return storage; }
	S & operator()(const size_t i)				{ return storage[i]; }
	const S & operator()(const size_t i) const	{ return storage[i]; }
	
	
	/***** operateurs ******/
	
#define ELEMENTWISE(_OP_) \
	Vector<S,dim> operator _OP_ (const Vector<S,dim> & other) const { \
		Vector<S,dim> result; \
		for (size_t i=0; i<dim; i++)	result(i) = storage(i) _OP_ other.storage(i); \
		return result; \
	}
	
	ELEMENTWISE(+)
	ELEMENTWISE(-)
	ELEMENTWISE(*)
	ELEMENTWISE(/)
#undef ELEMENTWISE
	
#define WITHSCALAR(_OP_) \
	friend Vector<S,dim> operator _OP_ (const S & other, const Vector<S,dim> & vec) { \
		Vector<S,dim> result; \
		for (size_t i=0; i<dim; i++)	result(i) = other _OP_ vec(i); \
		return result; \
	}
	
	WITHSCALAR(+)
	WITHSCALAR(-)
	WITHSCALAR(*)
	WITHSCALAR(/)
#undef WITHSCALAR
	
	S norm() {
		S sum = 0;
		for (size_t i=0; i<dim; i++)	sum += storage[i]*storage[i];
		return sqrt(sum);
	}
	Vector<S,dim> normalize() 		{ return (*this)/this->norm(); }
};

template<class S, size_t dim>
S dot(const Vector<S, dim> & a, const Vector<S, dim> & b) {
	S result = 0;
	for (size_t i=0; i<dim; i++)	result += a(i) * b(i);
	return result;
}

template<class S, size_t dim>
Vector<S, 3> cross(const Vector<S, 3> & a, const Vector<S, 3> & b) {
	return Vector<S,3> ({
		a(1)*b(2)-a(2)*b(1), 
		a(2)*b(0)-a(0)*b(1),
		a(0)*b(1)-a(1)*b(0)
	});
}




template<class S>
class View {
public:
	S * storage;
	size_t rows;
	size_t cols;
	
	View(S* storage, const size_t rows, const size_t cols) :storage(storage), rows(rows), cols(cols) {}
	
	View<S> & operator()(size_t i)				{ return View(storage + (rows*i), rows, 1); }
	const View<S> & operator()(size_t i) const	{ return View(storage + (rows*i), rows, 1); }
	S & operator()(size_t i, size_t j)				{ return *(storage + (rows*i + j)); }
	const S & operator()(size_t i, size_t j) const	{ return *(storage + (rows*i + j)); }
	
	View<S> & invert(int *err=nullptr) {
		View<S> & A = *this;
		S tmp;
		
		if (A.rows != A.cols)	{
			if(err)	*err = 2;
			return *this;
		}
		
		size_t dim = A.rows;
		int pivrow, pivrows[dim]; 	// keeps track of current pivot row and row swaps
		int i,j,k;
		
		for (k = 0; k < dim; k++)
		{
			// find pivot row, the row with biggest entry in current column
			tmp = 0;
			for (i = k; i < dim; i++)
			{
				if(fabs(A(i,k)) >= tmp) {
					tmp = fabs(A(i,k));
					pivrow = i;
				}
			}

			// check for singular matrix
			if (A(pivrow,k) == 0.0f)
				if(err)	{
					*err = 1;
					return *this;
				}

			// Execute pivot (row swap) if needed
			if (pivrow != k)
			{
				// swap row k with pivrow
				for (j = 0; j < dim; j++) {
					tmp = A(k,j);
					A(k,j) = A(pivrow,j);
					A(pivrow,j) = tmp;
				}
			}
			pivrows[k] = pivrow;	// record row swap (even if no swap happened)

			tmp = 1.0f / A(k,k);	// invert pivot element
			A(k,k) = 1.0f;		// This element of input matrix becomes result matrix

			// Perform row reduction (divide every element by pivot)
			for (j = 0; j < dim; j++)
				A(k,j) = A(k,j) * tmp;

			// Now eliminate all other entries in this column
			for (i = 0; i < dim; i++)
			{
				if (i != k) {
					tmp = A(i,k);
					A(i,k) = 0.0f;  // The other place where in matrix becomes result mat

					for (j = 0; j < dim; j++)
						A(i,j) = A(i,j) - A(k,j) * tmp;
				}
			}
		}

		// Done, now need to undo pivot row swaps by doing column swaps in reverse order
		for (k = dim-1; k >= 0; k--)
		{
			if (pivrows[k] != k) {
				for (i = 0; i < dim; i++) {
					tmp = A(i,k);
					A(i,k) = A(i,pivrows[k]);
					A(i,pivrows[k]) = tmp;
				}
			}
		}

		if (err)	*err = 0;
		return *this;
	}

};




template <class S, size_t rows, size_t cols>
struct Matrix {
	S storage[rows*cols];
	
public:
	// constructeurs
	Matrix() {}
	Matrix(const S v) {
		for (size_t i=0; i<rows*cols; i++) 		storage[i] = v;
	}
	Matrix(const Vector<S,rows> data[cols]) {
		for (size_t j=0; j<cols; j++)	{
			for (size_t i=0; i<rows; i++)	(*this)(i,j) = data[i];
		}
	}
	Matrix(const S * data) {
		for (size_t j=0; j<cols; j++) {
			for (size_t i=0; i<rows; i++)	(*this)(i,j) = data[i*cols + j];
		}
	}
	
	static Matrix<S,rows,cols> identity() {
		Matrix<S,rows,cols> result(0.);
		size_t mindim = rows>cols? cols:rows;
		for (size_t i=0; i<mindim; i++)		result(i,i) = 1;
		return result;
	}
	
	// methodes d'acces
	size_t nrows() const { return rows; }
	size_t ncols() const { return cols; }
	S * ptr() { return storage; }
	
	S & operator()(const size_t r, const size_t c)	{ return storage[r + rows*c]; }
	const S & operator()(const size_t r, const size_t c) const	{ return storage[r + rows*c]; }
	
	Vector<S, rows> & col(const size_t c) 				{ return * ((Vector<S,rows>*) (storage + (c*rows))); }
	const Vector<S, rows> & col(const size_t c) const 	{ return * ((const Vector<S,rows>*) (storage + (c*rows))); }
	Vector<S, rows> row(const size_t r) const { 
		Vector<S, rows> result;
		for (size_t i=0; i<cols; i++)	result[i] = (*this)(r,i);
		return result;
	}
	
	
	/***** operateurs ******/
	
	Vector<S, rows> operator*(const Vector<S, cols> & vec) const {
		Vector<S, rows> result;
		for (size_t i=0; i<rows; i++) {
			result(i) = 0.;
			for (size_t k=0; k<cols; k++)	result(i) += (*this)(i,k) * vec(k);
		}
		return result;
	}
	
	template<size_t ocols>
	Matrix<S, rows, ocols> operator*(const Matrix<S, cols, ocols> & other) const {
		Matrix<S, rows, ocols> result;
		for (size_t j=0; j<ocols; j++)		result.col(j) = (*this) * other.col(j);
		return result;
	}
	
#define ELEMENTWISE(_OP_) \
	Matrix<S, rows, cols> operator _OP_ (const Matrix<S, rows, cols> & other) const { \
		Matrix<S, rows, cols> result; \
		for (size_t i=0; i<rows; i++) { \
			for (size_t j=0; j<cols; j++) \
				result(i,j) = (*this)(i,j) _OP_ other(i,j); \
		} \
		return result; \
	}
	
	ELEMENTWISE(+)
	ELEMENTWISE(-)
#undef ELEMENTWISE

#define WITHSCALAR(_OP_) \
	friend Matrix<S, rows, cols> operator _OP_ (const S & other, const Matrix<S, rows, cols> & mat) { \
		Matrix<S, rows, cols> result; \
		for (size_t i=0; i<rows; i++) { \
			for (size_t j=0; j<cols; j++) \
				result(i,j) = other _OP_ mat(i,j); \
		} \
		return result; \
	}
	
	WITHSCALAR(+)
	WITHSCALAR(-)
	WITHSCALAR(*)
	WITHSCALAR(/)
#undef ELEMENTWISE
	
	Matrix<S,rows,cols> inverse(int *err=nullptr) {
		Matrix<S, rows, cols> result = *this;
		View<S>(result.storage, rows, cols).invert(err);
		return result;
	}
};


// definitions pratiques
typedef Vector<float, 2> vec2;
typedef Vector<float, 3> vec3;
typedef Vector<float, 4> vec4;
typedef Matrix<float, 2, 2> mat2;
typedef Matrix<float, 3, 3> mat3;
typedef Matrix<float, 4, 4> mat4;

typedef Vector<float, 8> vec8;
typedef Matrix<float, 8, 8> mat8;

};
#endif
