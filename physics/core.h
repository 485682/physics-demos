#pragma once

#include <cmath>

template < typename T>
struct _vector2{

	/* constructors ************************************************************/
	_vector2():x(0),y(0) {}
	_vector2(const T& v):x(v),y(v) {}
	_vector2(const _vector2 &v ):x(v.x),y(v.y) {}
	_vector2(const T& x_,const T& y_ ):x(x_),y(y_) {}
	/***************************************************************************/

	T& operator [](const int &index ) { return (&x)[ index ]; }
	const T& operator [](const int &index )const { return (&x)[ index ]; }

	/* copies this vector */
	void operator =(const _vector2 &v) { x=v.x; y=v.y; }

	/** 
	* flips all the components of the vector 
	*/
	void negate() { x=-x; y=-y; }

	T x,y;
};

template < typename T>
struct _vector3{

	/* constructors ************************************************************/

	_vector3():x( T(0.0f) ),y( T(0.0f) ),z( T(0.0f) ) {}

	/* initialize with scalar */
	_vector3(const T& s):x(s),y(s),z(s) {}

	/* copy */
	_vector3(const _vector3& v ):x(v.x),y(v.y),z(v.z) {}

	/* initialize components  */
	_vector3(const T& x_,const T& y_,const T& z_ ):x(x_),y(y_),z(z_) {}

	/***************************************************************************/

	/* copies this vector */
	void operator =(const _vector3<T> &v) { x=v.x; y=v.y; z=v.z; }

	T& operator [](const int &index ) { return (&x)[ index ]; }
	const T& operator [](const int &index )const { return (&x)[ index ]; }


	/** 
	* flips all the components of the vector 
	*/
	void invert() { x=-x; y=-y; z=-z; }

	/** 
	* multiply this vector with the given scalar 
	*/
	void operator *=(const T& s) { x*=s; y*=s; z*=s; }

	/** 
	* return this vector multiplied by the given scalar 
	*/
	_vector3 operator *(const T& s) const { return _vector3(x*s,y*s,z*s); }

	/** 
	* multiply this vector with the given vector 
	*/
	void operator *=(const _vector3& v) { x*=v.x; y*=v.y; z*=v.z; }

	/** 
	* return this vector multiplied by the given vector
	*/
	_vector3 operator *(const _vector3& v) const { return _vector3(x*v.x,y*v.y,z*v.z); }

	/** 
	* add the given vector to this vector 
	*/
	void operator +=( const _vector3& v ) { x+=v.x; y+=v.y; z+=v.z; }

	/** 
	* return this vector added to the given vector 
	*/
	_vector3 operator +(const  _vector3& v) const { return _vector3(x+v.x,y+v.y,z+v.z); }

	/** subtract the given vector from this vector */
	void operator -=(const _vector3& v) { x-=v.x; y-=v.y; z-=v.z; }

	/** 
	* return the given vector subtracted from this vector 
	*/
	_vector3 operator -(const  _vector3& v) const { return _vector3(x-v.x,y-v.y,z-v.z); }

	/** 
	* return the magnitude of this vector 
	*/
	T magnitude() const { return sqrt( (x*x)+(y*y)+(z*z) ); }

	/** 
	* return the squared magnitude of this vector 
	*/
	T squaremagnitude() const { return x*x+y*y+z*z; }

	/** 
	* limits the size of the vector to the given maximum 
	*/
	void trim(T size) {
		if (squaremagnitude() > size*size) {
			normalise();
			x *= size;
			y *= size;
			z *= size;
		}
	}

	/**
	* performs a component-wise product with the given vector and
	* sets this vector to its result
	*/
	void componentproductupdate(const _vector3<T> &v ) {
		x *= v.x;
		y *= v.y;
		z *= v.z;
	}

	/** 
	* turns a non-zero vector into a vector of unit length 
	*/
	void normalise() {
		const T len = magnitude();
		if (len > 0) {
			x = x/len;
			y = y/len;
			z = z/len;
		}
	}

	/** 
	* returns the normalised version of a vector 
	*/
	_vector3 normal() const {
		const T len = magnitude();
		if (len > 0) {
			return _vector3(x/len,y/len,z/len);
		}
		return *this;
	}

	/** 
	* zero all the components of the vector 
	*/
	void clear() { x = y = z = 0; } 

	/**
	* adds the given vector to this vector, scaled by the given amount
	*/
	void addscaledvector(const _vector3& vector, T scale) { 
		x += vector.x * scale;
		y += vector.y * scale;
		z += vector.z * scale;
	}

	T x,y,z;
};

/**
* returns the scalar product of v1 and v2
*/
template <typename T>
T _dot(const _vector3<T>&v1,const _vector3<T>&v2) { return (v1.x*v2.x+v1.y*v2.y+v1.z*v2.z); }

/**
* returns the vector product of v1 and v2
*/
template <typename T>
_vector3<T> _cross(const _vector3<T>&v1,const _vector3<T>&v2) {
	return _vector3<T>(  v1.y*v2.z-v1.z*v2.y,  v1.z*v2.x-v1.x*v2.z,  v1.x*v2.y-v1.y*v2.x );
}

/** 
* returns the normalised version of v 
*/
template <typename T>
_vector3<T> _normalize(const _vector3<T>&v) {
	const T len = v.magnitude();
	if(len>0){
		return _vector3<T>(v.x/len,v.y/len,v.z/len);
	}
	return v;
}

/** 
* returns the distance between two vectors as a scalar 
*/
template <typename T>
T _distance(const _vector3<T>&v1,const _vector3<T>&v2) {
	_vector3<T> dif = v1-v2;
	return dif.magnitude();
}

template < typename T>
struct _vector4{

	/* constructors ************************************************************/

	_vector4():x( T(0.0f) ),y( T(0.0f) ),z( T(0.0f) ),w( T(0.0f) ) {}

	/* initialize with scalar */
	_vector4(const T& v):x(v),y(v),z(v),w(v) {}

	/* copy */
	_vector4(const _vector4& v ):x(v.x),y(v.y),z(v.z),w(v.w) {}

	/* initialize components  */
	_vector4(const T& x_,const T& y_,const T& z_,const T& w_ ):x(x_),y(y_),z(z_),w(w_) {}

	/***************************************************************************/

	T& operator [](const int &index ) { return (&x)[ index ]; }
	const T& operator [](const int &index )const { return (&x)[ index ]; }

	/** 
	* copies this vector 
	*/
	void operator =(const _vector4 &v) { x=v.x; y=v.y; z=v.z; w=v.w; }

	/** 
	* flips all the components of the vector 
	*/
	void invert() { x=-x; y=-y; z=-z; w=-w; }

	/** 
	* multiply this vector with the given scalar 
	*/
	void operator *=(const T& s) { x*=s; y*=s; z*=s; w*=s; }

	/** 
	* return this vector multiplied by the given scalar 
	*/
	_vector4 operator *(const T& s) const { return _vector4(x*s,y*s,z*s,w*s); }

	/** 
	* multiply this vector with the given vector 
	*/
	void operator *=(const _vector4& v) { x*=v.x; y*=v.y; z*=v.z; w*=v.w; }

	/** 
	* return this vector multiplied by the given vector
	*/
	_vector4 operator *(const _vector4& v) const { return _vector4(x*v.x,y*v.y,z*v.z,w*v.w); }

	/** 
	* add the given vector to this vector 
	*/
	void operator +=(const _vector4& v) { x+=v.x; y+=v.y; z+=v.z; w+=v.w; }

	/** 
	* return this vector added to the given vector 
	*/
	_vector4 operator +(const  _vector4& v) const { return _vector4(x+v.x,y+v.y,z+v.z,w+v.w); }

	T x,y,z,w;
};

/**
* holds a three degree of freedom orientation.
*
* quaternions have
* several mathematical properties that make them useful for
* representing orientations, but require four items of data to
* hold the three degrees of freedom. these four items of data can
* be viewed as the coefficients of a complex number with three
* imaginary parts. the mathematics of the quaternion is then
* defined and is roughly correspondent to the math of 3d
* rotations. a quaternion is only a valid rotation if it is
* normalised: i.e. it has a length of 1.
*
* angular velocity and acceleration can be correctly
* represented as vectors. quaternions are only needed for
* orientation.
*/

class _quaternion { 

public:
	union {
		struct {
			/**
			* holds the real component of the quaternion.
			*/
			float r;
			/**
			* holds the first complex component of the
			* quaternion.
			*/
			float i;
			/**
			* holds the second complex component of the
			* quaternion.
			*/
			float j;
			/**
			* holds the third complex component of the
			* quaternion.
			*/
			float k;
		};
		/**
		* holds the quaternion data in array form.
		*/
		float data[4];
	};

	/**
	* the default constructor creates a quaternion representing
	* a zero rotation.
	*/
	_quaternion() : r(1), i(0), j(0), k(0) {}

	/**
	* the explicit constructor creates a quaternion with the given
	* components.
	*
	* the given orientation does not need to be normalised,
	* and can be zero. this function will not alter the given
	* values, or normalise the quaternion. to normalise the
	* quaternion (and make a zero quaternion a legal rotation),
	* use the normalise function.
	*
	*/
	_quaternion(const float r, const float i, const float j, const float k): r(r), i(i), j(j), k(k) { }

	/**
	* normalises the quaternion to unit length, making it a valid
	* orientation quaternion.
	*/
	void normalise() {
		float d = r*r+i*i+j*j+k*k;

		// check for zero length quaternion, and use the no-rotation
		// quaternion in that case.
		if (d < FLT_EPSILON ) { r = 1.0f; return; }

		d = ((float)1.0)/sqrt(d);
		r *= d;
		i *= d;
		j *= d;
		k *= d;
	}

	/**
	* multiplies the quaternion by the given quaternion
	*/
	void operator *=(const _quaternion &multiplier) {
		_quaternion q = *this;
		r = q.r*multiplier.r - q.i*multiplier.i -
			q.j*multiplier.j - q.k*multiplier.k;
		i = q.r*multiplier.i + q.i*multiplier.r +
			q.j*multiplier.k - q.k*multiplier.j;
		j = q.r*multiplier.j + q.j*multiplier.r +
			q.k*multiplier.i - q.i*multiplier.k;
		k = q.r*multiplier.k + q.k*multiplier.r +
			q.i*multiplier.j - q.j*multiplier.i;
	}

	/**
	* adds the given vector to this, scaled by the given amount.
	* this is used to update the orientation quaternion by a rotation
	* and time.
	*/
	void addscaledvector(const _vector3<float>& vector, float scale) {
		_quaternion q(0,
			vector.x * scale,
			vector.y * scale,
			vector.z * scale);
		q *= *this;
		r += q.r * ((float)0.5);
		i += q.i * ((float)0.5);
		j += q.j * ((float)0.5);
		k += q.k * ((float)0.5);
	}

	void rotatebyvector(const _vector3<float>& vector) {
		_quaternion q(0, vector.x, vector.y, vector.z);
		(*this) *= q;
	}
};


template < typename T>
struct _matrix3{

	/* constructors ************************************************************/

	/* creates an identity matrix*/
	_matrix3() { m_data[0][0]=m_data[1][1]=m_data[2][2]=1; }

	/**
	* sets the matrix to be a diagonal matrix with the given
	* values along the leading diagonal.
	*/
	_matrix3(const T& s) { m_data[0][0]=m_data[1][1]=m_data[2][2]=s; }

	/* copy */
	_matrix3(const _matrix3& m) {  operator=(m); }

	/**
	* creates a new matrix with explicit coefficients.
	*/
	_matrix3(float c0,float c1,float c2,float c3,
		float c4,float c5,float c6,float c7,float c8) {
			(*this)[0][0]=c0; (*this)[1][0]=c1; (*this)[2][0]=c2;
			(*this)[0][1]=c3; (*this)[1][1]=c4; (*this)[2][1]=c5;
			(*this)[0][2]=c6; (*this)[1][2]=c7; (*this)[2][2]=c8;
	}

	/***************************************************************************/

	/* copies this matrix */
	void operator=(const _matrix3 &m) {
		m_data[0]=m.m_data[0];
		m_data[1]=m.m_data[1];
		m_data[2]=m.m_data[2];
	}

	/**
	* multiplies this matrix in place by the given other matrix
	*/
	void operator*=(const _matrix3<T> &o) {
		T t1;
		T t2;
		T t3;

		t1 = m_data[0][0]*o[0][0] + m_data[1][0]*o[0][1] + m_data[2][0]*o[0][2];
		t2 = m_data[0][0]*o[1][0] + m_data[1][0]*o[1][1] + m_data[2][0]*o[1][2];
		t3 = m_data[0][0]*o[2][0] + m_data[1][0]*o[2][1] + m_data[2][0]*o[2][2];
		m_data[0][0] = t1;
		m_data[1][0] = t2;
		m_data[2][0] = t3;

		t1 = m_data[0][1]*o[0][0] + m_data[1][1]*o[0][1] + m_data[2][1]*o[0][2];
		t2 = m_data[0][1]*o[1][0] + m_data[1][1]*o[1][1] + m_data[2][1]*o[1][2];
		t3 = m_data[0][1]*o[2][0] + m_data[1][1]*o[2][1] + m_data[2][1]*o[2][2];
		m_data[0][1] = t1;
		m_data[1][1] = t2;
		m_data[2][1] = t3;

		t1 = m_data[0][2]*o[0][0] + m_data[1][2]*o[0][1] + m_data[2][2]*o[0][2];
		t2 = m_data[0][2]*o[1][0] + m_data[1][2]*o[1][1] + m_data[2][2]*o[1][2];
		t3 = m_data[0][2]*o[2][0] + m_data[1][2]*o[2][1] + m_data[2][2]*o[2][2];
		m_data[0][2] = t1;
		m_data[1][2] = t2;
		m_data[2][2] = t3;
	}

	/**
	* multiplies this matrix in place by the given scalar
	*/
	void operator*=(const float s) {
		m_data[0][0] *= s; m_data[1][0] *= s; m_data[2][0] *= s;
		m_data[0][1] *= s; m_data[1][1] *= s; m_data[2][1] *= s;
		m_data[0][2] *= s; m_data[1][2] *= s; m_data[2][2] *= s;
	}

	/**
	* does a component-wise addition of this matrix and the given matrix
	*/
	void operator+=(const _matrix3<T> &o) {
		m_data[0][0] += o[0][0]; m_data[1][0] += o[1][0]; m_data[2][0] += o[2][0];
		m_data[0][1] += o[0][1]; m_data[1][1] += o[1][1]; m_data[2][1] += o[2][1];
		m_data[0][2] += o[0][2]; m_data[1][2] += o[1][2]; m_data[2][2] += o[2][2];
	}

	/**
	* transform the given vector by this matrix
	*/
	_vector3<T> operator * (const _vector3<T>& v) const {
		return _vector3<T> (
			v.x*m_data[0][0] + v.y*m_data[1][0] + v.z*m_data[2][0],
			v.x*m_data[0][1] + v.y*m_data[1][1] + v.z*m_data[2][1],
			v.x*m_data[0][2] + v.y*m_data[1][2] + v.z*m_data[2][2]);
	}

	_vector3<T>& operator [](const int &index ) { return m_data[ index ]; }
	const _vector3<T>& operator [](const int &index )const { return m_data[ index ]; }


	/**
	* transform the given vector by this matrix
	*/
	_vector3<T> transform(const _vector3<T> &v) const { return (*this) * v; }

	/**
	* transform the given vector by the transpose of this matrix
	*/
	_vector3<T> transformtranspose( const _vector3<T> &v ) const {
		return _vector3<T>(
			v.x * m_data[0][0] + v.y * m_data[0][1] + v.z * m_data[0][2],
			v.x * m_data[1][0] + v.y * m_data[1][1] + v.z * m_data[1][2],
			v.x * m_data[2][0] + v.y * m_data[2][1] + v.z * m_data[2][2]
		);
	}

	/**
	* sets the matrix to be a diagonal matrix with the given
	* values along the leading diagonal.
	*/
	void setdiagonal(T a, T b, T c) { setinertiatensorcoeffs(a, b, c); }

	/**
	* sets the value of the matrix from inertia tensor values.
	*/
	void setinertiatensorcoeffs(T ix,T iy,T iz,T ixy=0,T ixz=0,T iyz=0) {
		(*this)[0][0] = ix;
		(*this)[1][0] = (*this)[0][1] = -ixy;
		(*this)[2][0] = (*this)[0][2] = -ixz;
		(*this)[1][1] = iy;
		(*this)[2][1] = (*this)[1][2] = -iyz;
		(*this)[2][2] = iz;
	}

	/**
	* sets the matrix values from the given three vector components
	* these are arranged as the three columns of the vector
	*/
	void setcomponents(const _vector3<T> &compone, const _vector3<T> &comptwo,const _vector3<T> &compthree) {
		m_data[0][0] = compone.x;
		m_data[1][0] = comptwo.x;
		m_data[2][0] = compthree.x;
		m_data[0][1] = compone.y;
		m_data[1][1] = comptwo.y;
		m_data[2][1] = compthree.y;
		m_data[0][2] = compone.z;
		m_data[1][2] = comptwo.z;
		m_data[2][2] = compthree.z;
	}

	/**
	* sets the matrix to be the inverse of the given matrix
	*/
	void setinverse(const _matrix3<T>& m) {
		float t4  = m[0][0]*m[1][1];
		float t6  = m[0][0]*m[2][1];
		float t8  = m[1][0]*m[0][1];
		float t10 = m[2][0]*m[0][1];
		float t12 = m[1][0]*m[0][2];
		float t14 = m[2][0]*m[0][2];

		// Calculate the determinant
		float t16 = ( t4*m[2][2]-t6*m[1][2]-t8*m[2][2]+t10*m[1][2]+t12*m[2][1]-t14*m[1][1] );

		// Make sure the determinant is non-zero.
		if (t16 == 0.0f) { return; }
		float t17 = 1/t16;

		(*this)[0][0] =  ( m[1][1]*m[2][2] - m[2][1]*m[1][2] )*t17;
		(*this)[1][0] = -( m[1][0]*m[2][2] - m[2][0]*m[1][2] )*t17;
		(*this)[2][0] =  ( m[1][0]*m[2][1] - m[2][0]*m[1][1] )*t17;
		(*this)[0][1] = -( m[0][1]*m[2][2] - m[2][1]*m[0][2] )*t17;
		(*this)[1][1] =  ( m[0][0]*m[2][2] - t14)*t17;
		(*this)[2][1] = -( t6-t10 )*t17;
		(*this)[0][2] =  ( m[0][1]*m[1][2] - m[1][1]*m[0][2] )*t17;
		(*this)[1][2] = -( m[0][0]*m[1][2] - t12)*t17;
		(*this)[2][2] =  (t4-t8)*t17;
	}

	/**
	* returns a new matrix containing the inverse of this matrix
	*/
	_matrix3<T> inverse() const {
		_matrix3<T> result;
		result.setinverse(*this);
		return result;
	}


	/**
	* sets the value of the matrix as an inertia tensor of
	* a rectangular block aligned with the body's coordinate
	* system with the given axis half-sizes and mass.
	*/
	void setblockinertiatensor(const _vector3<T> &halfsizes, float mass) {
		_vector3<T> squares = halfsizes * halfsizes;
		setinertiatensorcoeffs(0.3f*mass*(squares.y + squares.z),
			0.3f*mass*(squares.x + squares.z),
			0.3f*mass*(squares.x + squares.y));
	}

	/**
	* sets the matrix to be a skew symmetric matrix based on
	* the given vector. the skew symmetric matrix is the equivalent
	* of the vector product. so if a,b are vectors. a x b = a_s b
	* where a_s is the skew symmetric form of a
	*/
	void setskewsymmetric(const _vector3<T> v) {
		m_data[0][0] = m_data[1][1] = m_data[2][2] = 0;
		m_data[1][0] = -v.z;
		m_data[2][0] = v.y;
		m_data[0][1] = v.z;
		m_data[2][1] = -v.x;
		m_data[0][2] = -v.y;
		m_data[1][2] = v.x;
	}


	_vector3<T> m_data[3];

	/**
	* interpolates a couple of matrices.
	*/
	static _matrix3<T> linearinterpolate(const _matrix3<T>& a, const _matrix3<T>& b, T prop) {
		_matrix3<T> result;
		T * a_data      = (T*)a.m_data;
		T * b_data      = (T*)b.m_data;
		T * result_data = (T*)result.m_data;
		for (uint32_t i = 0; i < 9; i++) {
			result_data[i] = a_data[i] * (1-prop) + b_data[i] * prop;
		}
		return result;
	}

};

/** 
* returns matrix containing the transpose of matrix m
*/
template <typename T>
_matrix3<T> _transpose(const _matrix3<T>& m) {
	_matrix3<T> result;
	result[0][0]=m[0][0];  result[0][1]=m[1][0];  result[0][2]=m[2][0];
	result[1][0]=m[0][1];  result[1][1]=m[1][1];  result[1][2]=m[2][1];
	result[2][0]=m[0][2];  result[2][1]=m[1][2];  result[2][2]=m[2][2];
	return result;
}

/**
* returns matrix m1 and m2 multiplied 
*/
template <typename T>
_matrix3<T> operator*(const _matrix3<T>& m1,const _matrix3<T>& m2) {
	_matrix3<T> result;

	result[0][0] = m1[0][0]*m2[0][0] + m1[0][1]*m2[1][0] + m1[0][2]*m2[2][0];
	result[1][0] = m1[1][0]*m2[0][0] + m1[1][1]*m2[1][0] + m1[1][2]*m2[2][0];
	result[2][0] = m1[2][0]*m2[0][0] + m1[2][1]*m2[1][0] + m1[2][2]*m2[2][0];

	result[0][1] = m1[0][0]*m2[0][1] + m1[0][1]*m2[1][1] + m1[0][2]*m2[2][1];
	result[1][1] = m1[1][0]*m2[0][1] + m1[1][1]*m2[1][1] + m1[1][2]*m2[2][1];
	result[2][1] = m1[2][0]*m2[0][1] + m1[2][1]*m2[1][1] + m1[2][2]*m2[2][1];

	result[0][2] = m1[0][0]*m2[0][2] + m1[0][1]*m2[1][2] + m1[0][2]*m2[2][2];
	result[1][2] = m1[1][0]*m2[0][2] + m1[1][1]*m2[1][2] + m1[1][2]*m2[2][2];
	result[2][2] = m1[2][0]*m2[0][2] + m1[2][1]*m2[1][2] + m1[2][2]*m2[2][2];

	return result;
}

template < typename T>
struct _matrix4{

	/**constructors ************************************************************/
	/* creates an identity matrix*/
	_matrix4() { m_data[0][0]=m_data[1][1]=m_data[2][2]=m_data[3][3]=1; }
	/**
	* Sets the matrix to be a diagonal matrix with the given
	* values along the leading diagonal.
	*/
	_matrix4(const T& s) { m_data[0][0]=m_data[1][1]=m_data[2][2]=m_data[3][3]=s; }
	/* copy */
	_matrix4(const _matrix4& m) { operator=(m); }
	/***************************************************************************/

	/* copies this matrix */
	void operator=(const _matrix4 &m) {
		m_data[0]=m.m_data[0];
		m_data[1]=m.m_data[1];
		m_data[2]=m.m_data[2];
		m_data[3]=m.m_data[3];
	}

	/**
	* Returns a matrix which is this matrix multiplied by the given scalar
	*/
	_matrix4 operator*(const T& s) {
		_matrix4 result(*this);
		result[0]*=s;
		result[1]*=s;
		result[2]*=s;
		result[3]*=s;
		return result;
	}

	/**
	* Transform the given vector by this matrix
	*/
	_vector4<T> operator * (const _vector4<T>& v) {
		return _vector4<T> (
			v.x*m_data[0][0] + v.y*m_data[1][0] + v.z*m_data[2][0] + v.w*m_data[3][0],
			v.x*m_data[0][1] + v.y*m_data[1][1] + v.z*m_data[2][1] + v.w*m_data[3][1],
			v.x*m_data[0][2] + v.y*m_data[1][2] + v.z*m_data[2][2] + v.w*m_data[3][2],
			v.x*m_data[0][3] + v.y*m_data[1][3] + v.z*m_data[2][3] + v.w*m_data[3][3]);
	}

	/**
	* Transform the given vector by this matrix
	*/
	_vector3<T> operator*(const _vector3<T> & v) const {
		return _vector3<T>(
			v.x*m_data[0][0] + v.y*m_data[1][0] + v.z*m_data[2][0] + m_data[3][0],
			v.x*m_data[0][1] + v.y*m_data[1][1] + v.z*m_data[2][1] + m_data[3][1],
			v.x*m_data[0][2] + v.y*m_data[1][2] + v.z*m_data[2][2] + m_data[3][2]);
	}

	/**
	* Transform the given vector by this matrix
	*/
	_vector3<T> transform(const _vector3<T> &v) const { return (*this) * v; }

	_vector4<T>& operator [](const int &index ) { return m_data[ index ]; }
	const _vector4<T>& operator [](const int &index )const { return m_data[ index ]; }


	/**
	* transform the given vector by the transformational inverse
	* of this matrix.
	*
	* this function relies on the fact that the inverse of
	* a pure rotation matrix is its transpose. it separates the
	* translational and rotation components, transposes the
	* rotation, and multiplies out. if the matrix is not a
	* scale and shear free transform matrix, then this function
	* will not give correct results.
	*
	*/
	_vector3<T> transforminverse(const _vector3<T> &vector) const {
		_vector3<T> tmp = vector;
		tmp.x -= (*this)[3][0];
		tmp.y -= (*this)[3][1];
		tmp.z -= (*this)[3][2];
		return _vector3<T>(
			tmp.x * (*this)[0][0] +
			tmp.y * (*this)[0][1] +
			tmp.z * (*this)[0][2],

			tmp.x * (*this)[1][0] +
			tmp.y * (*this)[1][1] +
			tmp.z * (*this)[1][2],

			tmp.x * (*this)[2][0] +
			tmp.y * (*this)[2][1] +
			tmp.z * (*this)[2][2]
		);
	}

	/**
	* gets a vector representing one axis (i.e. one column) in the matrix
	*/
	_vector3<T> getaxisvector(uint32_t i) const { return _vector3<T>(m_data[i].x,m_data[i].y,m_data[i].z); }

	/**
	* transform the given direction vector by the
	* transformational inverse of this matrix.
	*
	* this function relies on the fact that the inverse of
	* a pure rotation matrix is its transpose. it separates the
	* translational and rotation components, transposes the
	* rotation, and multiplies out. if the matrix is not a
	* scale and shear free transform matrix, then this function
	* will not give correct results.
	*/
	_vector3<T> transforminversedirection(const _vector3<T> &v) const {
		return _vector3<T>(
			v.x * (*this)[0][0] + v.y * (*this)[0][1] + v.z * (*this)[0][2],
			v.x * (*this)[1][0] + v.y * (*this)[1][1] + v.z * (*this)[1][2],
			v.x * (*this)[2][0] + v.y * (*this)[2][1] + v.z * (*this)[2][2]
		);
	}

	/**
	* transform the given direction vector by this matrix
	*/
	_vector3<T> transformdirection(const _vector3<T> &v) const {
		return _vector3<T> (
			v.x * (*this)[0][0] + v.y * (*this)[1][0] + v.z * (*this)[2][0],
			v.x * (*this)[0][1] + v.y * (*this)[1][1] + v.z * (*this)[2][1],
			v.x * (*this)[0][2] + v.y * (*this)[1][2] + v.z * (*this)[2][2]
		);
	}


	_vector4<T> m_data[4];
};

/* returns matrix containing the transpose of matrix m*/
template <typename T>
_matrix4<T> _transpose(const _matrix4<T>& m) {
	_matrix4<T> result;
	result[0][0]=m[0][0];  result[0][1]=m[1][0];  result[0][2]=m[2][0]; result[0][3]=m[3][0];
	result[1][0]=m[0][1];  result[1][1]=m[1][1];  result[1][2]=m[2][1]; result[1][3]=m[3][1];
	result[2][0]=m[0][2];  result[2][1]=m[1][2];  result[2][2]=m[2][2]; result[2][3]=m[3][2];
	result[3][0]=m[0][3];  result[3][1]=m[1][3];  result[3][2]=m[2][3]; result[3][3]=m[3][3];
	return result;
}

/**
* returns matrix m1 and m2 multiplied 
*/
template <typename T>
_matrix4<T> operator*(const _matrix4<T>& m1,const _matrix4<T>& m2) {
	_matrix4<T> result;

	result[0][0] = m1[0][0]*m2[0][0] + m1[0][1]*m2[1][0] + m1[0][2]*m2[2][0]  + m1[0][3]*m2[3][0];
	result[1][0] = m1[1][0]*m2[0][0] + m1[1][1]*m2[1][0] + m1[1][2]*m2[2][0]  + m1[1][3]*m2[3][0];
	result[2][0] = m1[2][0]*m2[0][0] + m1[2][1]*m2[1][0] + m1[2][2]*m2[2][0]  + m1[2][3]*m2[3][0];
	result[3][0] = m1[3][0]*m2[0][0] + m1[3][1]*m2[1][0] + m1[3][2]*m2[2][0]  + m1[3][3]*m2[3][0];

	result[0][1] = m1[0][0]*m2[0][1] + m1[0][1]*m2[1][1] + m1[0][2]*m2[2][1]  + m1[0][3]*m2[3][1];
	result[1][1] = m1[1][0]*m2[0][1] + m1[1][1]*m2[1][1] + m1[1][2]*m2[2][1]  + m1[1][3]*m2[3][1];
	result[2][1] = m1[2][0]*m2[0][1] + m1[2][1]*m2[1][1] + m1[2][2]*m2[2][1]  + m1[2][3]*m2[3][1];
	result[3][1] = m1[3][0]*m2[0][1] + m1[3][1]*m2[1][1] + m1[3][2]*m2[2][1]  + m1[3][3]*m2[3][1];

	result[0][2] = m1[0][0]*m2[0][2] + m1[0][1]*m2[1][2] + m1[0][2]*m2[2][2]  + m1[0][3]*m2[3][2];
	result[1][2] = m1[1][0]*m2[0][2] + m1[1][1]*m2[1][2] + m1[1][2]*m2[2][2]  + m1[1][3]*m2[3][2];
	result[2][2] = m1[2][0]*m2[0][2] + m1[2][1]*m2[1][2] + m1[2][2]*m2[2][2]  + m1[2][3]*m2[3][2];
	result[3][2] = m1[3][0]*m2[0][2] + m1[3][1]*m2[1][2] + m1[3][2]*m2[2][2]  + m1[3][3]*m2[3][2];

	result[0][3] = m1[0][0]*m2[0][3] + m1[0][1]*m2[1][3] + m1[0][2]*m2[2][3]  + m1[0][3]*m2[3][3];
	result[1][3] = m1[1][0]*m2[0][3] + m1[1][1]*m2[1][3] + m1[1][2]*m2[2][3]  + m1[1][3]*m2[3][3];
	result[2][3] = m1[2][0]*m2[0][3] + m1[2][1]*m2[1][3] + m1[2][2]*m2[2][3]  + m1[2][3]*m2[3][3];
	result[3][3] = m1[3][0]*m2[0][3] + m1[3][1]*m2[1][3] + m1[3][2]*m2[2][3]  + m1[3][3]*m2[3][3];
	return result;
}

template <typename T> /*glm*/
_matrix4<T> _lookatrh (const _vector3<T>& eye,const _vector3<T>& center,const _vector3<T>& up) {
	const _vector3<T> f(_normalize(center - eye));
	const _vector3<T> s(_normalize(_cross(f, up)));
	const _vector3<T> u(_cross(s, f));

	_matrix4<T> result;
	result[0][0] = s.x;
	result[1][0] = s.y;
	result[2][0] = s.z;
	result[0][1] = u.x;
	result[1][1] = u.y;
	result[2][1] = u.z;
	result[0][2] =-f.x;
	result[1][2] =-f.y;
	result[2][2] =-f.z;
	result[3][0] =-_dot(s, eye);
	result[3][1] =-_dot(u, eye);
	result[3][2] = _dot(f, eye);
	return result;
}
template <typename T> /*glm*/
_matrix4<T> _perspectivefovrh(T fov,T width,T height,T zNear,T zFar ) {

	T const rad = fov;
	T const h = cos(T(0.5) * rad) / sin(T(0.5) * rad);
	T const w = h * height / width; ///todo max(width , Height) / min(width , Height)?

	_matrix4<T> result(T(0));
	result[0][0] = w;
	result[1][1] = h;
	result[2][2] = - (zFar + zNear) / (zFar - zNear);
	result[2][3] = - T(1);
	result[3][2] = - (T(2) * zFar * zNear) / (zFar - zNear);
	return result;
}
template <typename T> /*glm*/
_matrix4<T> _translate(const _vector3<T> & v ) {
	_matrix4<T> result;
	result[3] = result[0]*v[0] + result[1]*v[1] + result[2]*v[2] + result[3];
	return result;
}
template <typename T> /*glm*/
_matrix4<T> _scale ( const _vector3<T>& v ) {
	_matrix4<T> result,m;
	result[0] = m[0] * v[0];
	result[1] = m[1] * v[1];
	result[2] = m[2] * v[2];
	result[3] = m[3];
	return result;
}
template <typename T> /*glm*/
_matrix4<T> _rotate ( T angle, const _vector3<T>& v ) {

	const T a = angle;
	const T c = cos(a);
	const T s = sin(a);

	_vector3<T> axis(_normalize(v));
	_vector3<T> temp( axis*(T(1) - c));

	_matrix4<T> result;
	result[0][0] = c + temp[0] * axis[0];
	result[0][1] = 0 + temp[0] * axis[1] + s * axis[2];
	result[0][2] = 0 + temp[0] * axis[2] - s * axis[1];

	result[1][0] = 0 + temp[1] * axis[0] - s * axis[2];
	result[1][1] = c + temp[1] * axis[1];
	result[1][2] = 0 + temp[1] * axis[2] + s * axis[0];

	result[2][0] = 0 + temp[2] * axis[0] + s * axis[1];
	result[2][1] = 0 + temp[2] * axis[1] - s * axis[0];
	result[2][2] = c + temp[2] * axis[2];

	return result;
}
template <typename T> /*glm*/
_matrix4<T> _rotate ( const _matrix4<T>& m, T angle, const _vector3<T> & v ) {
	T const a = angle;
	T const c = cos(a);
	T const s = sin(a);

	_vector3<T> axis(_normalize(v));
	_vector3<T> temp(axis* (T(1) - c) );

	_matrix4<T> rotate(0);
	rotate[0][0] = c + temp[0] * axis[0];
	rotate[0][1] = 0 + temp[0] * axis[1] + s * axis[2];
	rotate[0][2] = 0 + temp[0] * axis[2] - s * axis[1];

	rotate[1][0] = 0 + temp[1] * axis[0] - s * axis[2];
	rotate[1][1] = c + temp[1] * axis[1];
	rotate[1][2] = 0 + temp[1] * axis[2] + s * axis[0];

	rotate[2][0] = 0 + temp[2] * axis[0] + s * axis[1];
	rotate[2][1] = 0 + temp[2] * axis[1] - s * axis[0];
	rotate[2][2] = c + temp[2] * axis[2];

	_matrix4<T> result(0);
	result[0] = m[0] * rotate[0][0] + m[1] * rotate[0][1] + m[2] * rotate[0][2];
	result[1] = m[0] * rotate[1][0] + m[1] * rotate[1][1] + m[2] * rotate[1][2];
	result[2] = m[0] * rotate[2][0] + m[1] * rotate[2][1] + m[2] * rotate[2][2];
	result[3] = m[3];
	return result;
}
template <typename T> /*glm*/
_matrix4<T> _yawpitchroll (const T&yaw, const T&pitch,const T&roll) {
	T tmp_ch = cos(yaw);
	T tmp_sh = sin(yaw);
	T tmp_cp = cos(pitch);
	T tmp_sp = sin(pitch);
	T tmp_cb = cos(roll);
	T tmp_sb = sin(roll);

	_matrix4<T> result;
	result[0][0] = tmp_ch * tmp_cb + tmp_sh * tmp_sp * tmp_sb;
	result[0][1] = tmp_sb * tmp_cp;
	result[0][2] = -tmp_sh * tmp_cb + tmp_ch * tmp_sp * tmp_sb;
	result[0][3] = static_cast<T>(0);
	result[1][0] = -tmp_ch * tmp_sb + tmp_sh * tmp_sp * tmp_cb;
	result[1][1] = tmp_cb * tmp_cp;
	result[1][2] = tmp_sb * tmp_sh + tmp_ch * tmp_sp * tmp_cb;
	result[1][3] = static_cast<T>(0);
	result[2][0] = tmp_sh * tmp_cp;
	result[2][1] = -tmp_sp;
	result[2][2] = tmp_ch * tmp_cp;
	result[2][3] = static_cast<T>(0);
	result[3][0] = static_cast<T>(0);
	result[3][1] = static_cast<T>(0);
	result[3][2] = static_cast<T>(0);
	result[3][3] = static_cast<T>(1);
	return result;
}

