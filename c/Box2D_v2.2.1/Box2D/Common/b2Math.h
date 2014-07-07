/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_MATH_H
#define B2_MATH_H

#include <Box2D/Common/b2Settings.h>

#include <cmath>
#include <cfloat>
#include <cstddef>
#include <limits>

#if defined(SIMD)
typedef float32 float32x4 __attribute__ ((vector_size (16), aligned(1)));
typedef int32 int32x4 __attribute__ ((vector_size (16), aligned(1)));
#endif

/// This function is used to ensure that a floating point number is
/// not a NaN or infinity.
inline bool b2IsValid(float32 x)
{
	if (x != x)
	{
		// NaN.
		return false;
	}

	float32 infinity = std::numeric_limits<float32>::infinity();
	return -infinity < x && x < infinity;
}

/// This is a approximate yet fast inverse square-root.
inline float32 b2InvSqrt(float32 x)
{
	union
	{
		float32 x;
		int32 i;
	} convert;

	convert.x = x;
	float32 xhalf = 0.5f * x;
	convert.i = 0x5f3759df - (convert.i >> 1);
	x = convert.x;
	x = x * (1.5f - xhalf * x * x);
	return x;
}

#define	b2Sqrt(x)	std::sqrt(x)
#define	b2Atan2(y, x)	std::atan2(y, x)

/// A 2D column vector.
struct b2Vec2
{
	/// Default constructor does nothing (for performance).
	b2Vec2() {}

	/// Construct using coordinates.
	b2Vec2(float32 x, float32 y)
#if !defined(SIMD)
	 : m_x(x), m_y(y) {}
#else
	{
		float32x4 f4 = {x, y, 0.0, 0.0};
		m_float4 = f4;
	}
#endif

#if defined(SIMD)
	explicit b2Vec2(float32x4 f4) : m_float4(f4) {}
#endif

	/// Set this vector to all zeros.
	void SetZero()
	{
#if !defined(SIMD)
		m_x = 0.0f; m_y = 0.0f;
#else
	    float32x4 z4 = {0.0f, 0.0f, 0.0f, 0.0f}; m_float4 = z4;
#endif
	}

	/// Set this vector to some specified coordinates.
	void Set(float32 x_, float32 y_) 
	{
#if !defined(SIMD)
		m_x = x_; m_y = y_;
#else
		float32x4 f4 = {x_, y_, 0.0, 0.0};
		m_float4 = f4;
#endif
	}

	inline float32 x() const
	{
#if !defined(SIMD)
		return m_x;
#else
		return m_float4[0];
#endif
	}
	inline float32 y() const
	{
#if !defined(SIMD)
		return m_y;
#else
		return m_float4[1];
#endif
	}
	inline void set_x(float32 x) 
	{
#if !defined(SIMD)
		m_x = x;
#else
		m_float4[0] = x;
#endif
	}
	inline void set_y(float32 y)
	{
#if !defined(SIMD)
		m_y = y;
#else
		m_float4[1] = y;
#endif
	}

	/// Negate this vector.
	b2Vec2 operator -() const 
	{ 
#if !defined(SIMD)
		b2Vec2 v; v.Set(-m_x, -m_y); return v;
#else 
		return b2Vec2(-m_float4); 
#endif
	}

	/// Read from and indexed element.
	float32 operator () (int32 i) const
	{
#if !defined(SIMD)
		return (&m_x)[i];
#else
		return m_float4[i];
#endif
	}

	void SetElement(int32 i, float32 f)
	{
#if !defined(SIMD)
		(&m_x)[i] = f;
#else	
		m_float4[i] = f;
#endif
	}

	/// Add a vector to this vector.
	void operator += (const b2Vec2& v)
	{
#if !defined(SIMD)
		m_x += v.m_x; m_y += v.m_y;
#else
		m_float4 += v.m_float4;
#endif
	}
	
	/// Subtract a vector from this vector.
	void operator -= (const b2Vec2& v)
	{
#if !defined(SIMD)
		m_x -= v.m_x; m_y -= v.m_y;
#else
		m_float4 -= v.m_float4;
#endif
	}

	/// Multiply this vector by a scalar.
	void operator *= (float32 a)
	{
#if !defined(SIMD)
		m_x *= a; m_y *= a;
#else
		float32x4 a4 = {a, a, 0.0, 0.0};
		m_float4 *= a4;
#endif
	}

	/// Get the length of this vector (the norm).
	float32 Length() const
	{
#if !defined(SIMD)
		return b2Sqrt(m_x * m_x + m_y * m_y);
#else
		float32x4 f4 = m_float4 * m_float4;
		return b2Sqrt(f4[0] + f4[1]);
#endif
	}

	/// Get the length squared. For performance, use this instead of
	/// b2Vec2::Length (if possible).
	float32 LengthSquared() const
	{
#if !defined(SIMD)
		return m_x * m_x + m_y * m_y;
#else
		float32x4 f4 = m_float4 * m_float4;
		return f4[0] + f4[1];
#endif
	}

	/// Convert this vector into a unit vector. Returns the length.
	float32 Normalize()
	{
		float32 length = Length();
		if (length < b2_epsilon)
		{
			return 0.0f;
		}
		float32 invLength = 1.0f / length;
#if !defined(SIMD)
		m_x *= invLength;
		m_y *= invLength;
#else
		float32x4 invLength4 = {invLength, invLength, 0.0, 0.0};
		m_float4 *= invLength4;
#endif
		return length;
	}

	/// Does this vector contain finite coordinates?
	bool IsValid() const
	{
		return b2IsValid(x()) && b2IsValid(y());
	}

	/// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
	b2Vec2 Skew() const
	{
		return b2Vec2(-y(), x());
	}

#if !defined(SIMD)
	float32 m_x, m_y;
#else
    float32x4 m_float4;
#endif
};

/// A 2D column vector with 3 elements.
struct b2Vec3
{
	/// Default constructor does nothing (for performance).
	b2Vec3() {}

	/// Construct using coordinates.
	b2Vec3(float32 x, float32 y, float32 z) 
#if !defined(SIMD)
	: m_x(x), m_y(y), m_z(z) {}
#else
	{
		m_float4[0] = x;
	    m_float4[1] = y;
	    m_float4[2] = z;
	    m_float4[3] = 0.0;
	}
#endif

#if defined(SIMD)
	explicit b2Vec3(float32x4 f4) : m_float4(f4) {}
#endif

	/// Set this vector to all zeros.
	void SetZero() 
	{ 
#if !defined(SIMD)
		m_x = 0.0f; m_y = 0.0f; m_z = 0.0f;
#else
		float32x4 z4 = {0.0, 0.0, 0.0, 0.0};
		m_float4 = z4;
#endif
	}

	/// Set this vector to some specified coordinates.
	void Set(float32 x_, float32 y_, float32 z_) 
	{ 
#if !defined(SIMD)
		m_x = x_; m_y = y_; m_z = z_;
#else
		m_float4[0] = x_;
	    m_float4[1] = y_;
	    m_float4[2] = z_;
	    m_float4[3] = 0.0;
#endif
	}

	/// Negate this vector.
	b2Vec3 operator -() const 
	{ 
#if !defined(SIMD)
		b2Vec3 v; v.Set(-m_x, -m_y, -m_z); return v;
#else
		return b2Vec3(-m_float4);
#endif
	}

	/// Add a vector to this vector.
	void operator += (const b2Vec3& v)
	{
#if !defined(SIMD)
		m_x += v.m_x; m_y += v.m_y; m_z += v.m_z;
#else
		m_float4 += v.m_float4;
#endif
	}

	/// Subtract a vector from this vector.
	void operator -= (const b2Vec3& v)
	{
#if !defined(SIMD)
		m_x -= v.m_x; m_y -= v.m_y; m_z -= v.m_z;
#else
		m_float4 -= v.m_float4;
#endif
	}

	/// Multiply this vector by a scalar.
	void operator *= (float32 s)
	{
#if !defined(SIMD)
		m_x *= s; m_y *= s; m_z *= s;
#else
		float32x4 s4 = {s, s, s, 0.0};
		m_float4 *= s4;
#endif
	}

	inline float32 x() const 
	{
#if !defined(SIMD)
		return m_x;
#else
		return m_float4[0];
#endif
	}
	inline float32 y() const 
	{
#if !defined(SIMD)
		return m_y;
#else
		return m_float4[1];
#endif
	}
	inline float32 z() const 
	{
#if !defined(SIMD)
		return m_z;
#else
		return m_float4[2];
#endif
	}
	inline void set_x(float32 x) 
	{
#if !defined(SIMD)
		m_x = x;
#else
		m_float4[0] = x;
#endif
	}
	inline void set_y(float32 y) 
	{
#if !defined(SIMD)
		m_y = y;
#else
		m_float4[1] = y;
#endif
	}
	inline void set_z(float32 z)
	{
#if !defined(SIMD)
		m_z = z;
#else
		m_float4[2] = z;
#endif
	}

#if !defined(SIMD)
	float32 m_x, m_y, m_z;
#else
	float32x4 m_float4;
#endif
};

/// A 2-by-2 matrix. Stored in column-major order.
struct b2Mat22
{
	/// The default constructor does nothing (for performance).
	b2Mat22() {}

	/// Construct this matrix using columns.
	b2Mat22(const b2Vec2& c1, const b2Vec2& c2)
	{
		ex = c1;
		ey = c2;
	}

	/// Construct this matrix using scalars.
	b2Mat22(float32 a11, float32 a12, float32 a21, float32 a22)
	{
		ex.set_x(a11); ex.set_y(a21);
		ey.set_x(a12); ey.set_y(a22);
	}

	/// Initialize this matrix using columns.
	void Set(const b2Vec2& c1, const b2Vec2& c2)
	{
		ex = c1;
		ey = c2;
	}

	/// Set this to the identity matrix.
	void SetIdentity()
	{
#if !defined(SIMD)
		ex.m_x = 1.0f; ey.m_x = 0.0f;
		ex.m_y = 0.0f; ey.m_y = 1.0f;
#else
		float32x4 ex4 = {1.0f, 0.0f, 0.0f, 0.0f};
		float32x4 ey4 = {0.0f, 1.0f, 0.0f, 0.0f};
		ex.m_float4 = ex4;
		ey.m_float4 = ey4;
#endif
	}

	/// Set this matrix to all zeros.
	void SetZero()
	{
#if !defined(SIMD)
		ex.m_x = 0.0f; ey.m_x = 0.0f;
		ex.m_y = 0.0f; ey.m_y = 0.0f;
#else
		float32x4 z4 = {0.0f, 0.0f, 0.0f, 0.0f};
		ex.m_float4 = z4;
		ey.m_float4 = z4;
#endif
	}

	b2Mat22 GetInverse() const
	{
		float32 a = ex.x(), b = ey.x(), c = ex.y(), d = ey.y();
		b2Mat22 B;
		float32 det = a * d - b * c;
		if (det != 0.0f)
		{
			det = 1.0f / det;
		}
		B.ex.set_x(det * d);	  B.ey.set_x(-det * b);
		B.ex.set_y(-det * c);     B.ey.set_y(det * a);
		return B;
	}

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	b2Vec2 Solve(const b2Vec2& b) const
	{
		float32 a11 = ex.x(), a12 = ey.x(), a21 = ex.y(), a22 = ey.y();
		float32 det = a11 * a22 - a12 * a21;
		if (det != 0.0f)
		{
			det = 1.0f / det;
		}
		b2Vec2 x;
		x.set_x(det * (a22 * b.x() - a12 * b.y()));
		x.set_y(det * (a11 * b.y() - a21 * b.x()));
		return x;
	}

	b2Vec2 ex, ey;
};

/// A 3-by-3 matrix. Stored in column-major order.
struct b2Mat33
{
	/// The default constructor does nothing (for performance).
	b2Mat33() {}

	/// Construct this matrix using columns.
	b2Mat33(const b2Vec3& c1, const b2Vec3& c2, const b2Vec3& c3)
	{
		ex = c1;
		ey = c2;
		ez = c3;
	}

	/// Set this matrix to all zeros.
	void SetZero()
	{
		ex.SetZero();
		ey.SetZero();
		ez.SetZero();
	}

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	b2Vec3 Solve33(const b2Vec3& b) const;

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases. Solve only the upper
	/// 2-by-2 matrix equation.
	b2Vec2 Solve22(const b2Vec2& b) const;

	/// Get the inverse of this matrix as a 2-by-2.
	/// Returns the zero matrix if singular.
	void GetInverse22(b2Mat33* M) const;

	/// Get the symmetric inverse of this matrix as a 3-by-3.
	/// Returns the zero matrix if singular.
	void GetSymInverse33(b2Mat33* M) const;

	b2Vec3 ex, ey, ez;
};

/// Rotation
struct b2Rot
{
	b2Rot() {}

#if defined(SIMD)
	explicit b2Rot(float32x4 f4) : m_float4(f4) {}

	explicit b2Rot(float32 s, float32 c) 
	{
		set_s(s);
		set_c(c);
	}
#endif

	/// Initialize from an angle in radians
	explicit b2Rot(float32 angle)
	{
		/// TODO_ERIN optimize
		set_s(sinf(angle));
		set_c(cosf(angle));
	}

	/// Set using an angle in radians.
	void Set(float32 angle)
	{
		/// TODO_ERIN optimize
		set_s(sinf(angle));
		set_c(cosf(angle));
	}

	/// Set to the identity rotation
	void SetIdentity()
	{
#if !defined(SIMD)
		m_s = 0.0f;
		m_c = 1.0f;
#else
		float32x4 i4 = {0.0f, 1.0f, 0.0f, 0.0f};
		m_float4 = i4;
#endif
	}

	/// Get the angle in radians
	float32 GetAngle() const
	{
		return b2Atan2(s(), c());
	}

	/// Get the x-axis
	b2Vec2 GetXAxis() const
	{
		return b2Vec2(c(), s());
	}

	/// Get the u-axis
	b2Vec2 GetYAxis() const
	{
		return b2Vec2(-s(), c());
	}

	/// Sine and cosine
	inline float32 s() const 
	{
#if !defined(SIMD)
		return m_s;
#else
		return m_float4[0];
#endif
	}
	inline float32 c() const 
	{
#if !defined(SIMD)
		return m_c;
#else
		return m_float4[1];
#endif
	}
	inline void set_s(float32 s) 
	{
#if !defined(SIMD)
		m_s = s;
#else
		m_float4[0] = s;
#endif
	}
	inline void set_c(float32 c) 
	{
#if !defined(SIMD)
		m_c = c;
#else
		m_float4[1] = c;
#endif
	}

#if !defined(SIMD)
	float32 m_s, m_c;
#else
	float32x4 m_float4;
#endif
};

/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
struct b2Transform
{
	/// The default constructor does nothing.
	b2Transform() {}

	/// Initialize using a position vector and a rotation.
	b2Transform(const b2Vec2& position, const b2Rot& rotation) : p(position), q(rotation) {}

	/// Set this to the identity transform.
	void SetIdentity()
	{
		p.SetZero();
		q.SetIdentity();
	}

	/// Set this based on the position and angle.
	void Set(const b2Vec2& position, float32 angle)
	{
		p = position;
		q.Set(angle);
	}

	b2Vec2 p;
	b2Rot q;
};

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
struct b2Sweep
{
	/// Get the interpolated transform at a specific time.
	/// @param beta is a factor in [0,1], where 0 indicates alpha0.
	void GetTransform(b2Transform* xfb, float32 beta) const;

	/// Advance the sweep forward, yielding a new initial state.
	/// @param alpha the new initial time.
	void Advance(float32 alpha);

	/// Normalize the angles.
	void Normalize();

	b2Vec2 localCenter;	///< local center of mass position
	b2Vec2 c0, c;		///< center world positions
	float32 a0, a;		///< world angles

	/// Fraction of the current time step in the range [0,1]
	/// c0 and a0 are the positions at alpha0.
	float32 alpha0;
};

/// Useful constant
extern const b2Vec2 b2Vec2_zero;

/// Perform the dot product on two vectors.
inline float32 b2Dot(const b2Vec2& a, const b2Vec2& b)
{
#if !defined(SIMD)
	return a.m_x * b.m_x + a.m_y * b.m_y;
#else
	float32x4 c4 = a.m_float4 * b.m_float4;
	return c4[0] + c4[1];
#endif
}

/// Perform the cross product on two vectors. In 2D this produces a scalar.
inline float32 b2Cross(const b2Vec2& a, const b2Vec2& b)
{
#if !defined(SIMD)
	return a.m_x * b.m_y - a.m_y * b.m_x;
#else
    float32x4 c4 = a.m_float4 * __builtin_shufflevector(b.m_float4, b.m_float4, 1, 0, -1, -1);
	return c4[0] - c4[1];
#endif
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
inline b2Vec2 b2Cross(const b2Vec2& a, float32 s)
{
#if !defined(SIMD)
	return b2Vec2(s * a.m_y, -s * a.m_x);
#else
	float32x4 s4 = {s, s, 0.0, 0.0};
	float32x4 f4 = a.m_float4 * s4;
	return b2Vec2(f4[1], -f4[0]);
#endif
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
inline b2Vec2 b2Cross(float32 s, const b2Vec2& a)
{
#if !defined(SIMD)
	return b2Vec2(-s * a.m_y, s * a.m_x);
#else
	float32x4 s4 = {s, s, 0.0, 0.0};
	float32x4 f4 = a.m_float4 * s4;
	return b2Vec2(-f4[1], f4[0]);
#endif
}

/// Multiply a matrix times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another.
inline b2Vec2 b2Mul(const b2Mat22& A, const b2Vec2& v)
{
#if !defined(SIMD)
	return b2Vec2(A.ex.m_x * v.m_x + A.ey.m_x * v.m_y, 
	              A.ex.m_y * v.m_x + A.ey.m_y * v.m_y);
#else
	float32x4 f4 = A.ex.m_float4 * __builtin_shufflevector(v.m_float4, v.m_float4, 0, 0, -1, -1);
	f4 += A.ey.m_float4 * __builtin_shufflevector(v.m_float4, v.m_float4, 1, 1, -1, -1);
	return b2Vec2(f4);
#endif
}

/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
inline b2Vec2 b2MulT(const b2Mat22& A, const b2Vec2& v)
{
	return b2Vec2(b2Dot(v, A.ex), b2Dot(v, A.ey));
}

/// Add two vectors component-wise.
inline b2Vec2 operator + (const b2Vec2& a, const b2Vec2& b)
{
#if !defined(SIMD)
	return b2Vec2(a.m_x + b.m_x, a.m_y + b.m_y);
#else
	return b2Vec2(a.m_float4 + b.m_float4);
#endif
}

/// Subtract two vectors component-wise.
inline b2Vec2 operator - (const b2Vec2& a, const b2Vec2& b)
{
#if !defined(SIMD)
	return b2Vec2(a.m_x - b.m_x, a.m_y - b.m_y);
#else
	return b2Vec2(a.m_float4 - b.m_float4);
#endif
}

inline b2Vec2 operator * (float32 s, const b2Vec2& a)
{
#if !defined(SIMD)
	return b2Vec2(s * a.m_x, s * a.m_y);
#else
	float32x4 s4 = {s, s, 0.0, 0.0};
	return b2Vec2(a.m_float4 * s4);
#endif
}

inline bool operator == (const b2Vec2& a, const b2Vec2& b)
{
#if !defined(SIMD)
	return a.m_x == b.m_x && a.m_y == b.m_y;
#else
	int32x4 i4 = a.m_float4 == b.m_float4;
	return i4[0] && i4[1];
#endif
}

inline float32 b2Distance(const b2Vec2& a, const b2Vec2& b)
{
	b2Vec2 c = a - b;
	return c.Length();
}

inline float32 b2DistanceSquared(const b2Vec2& a, const b2Vec2& b)
{
	b2Vec2 c = a - b;
	return b2Dot(c, c);
}

inline b2Vec3 operator * (float32 s, const b2Vec3& a)
{
#if !defined(SIMD)
	return b2Vec3(s * a.m_x, s * a.m_y, s * a.m_z);
#else
	float32x4 s4 = {s, s, s, 0.0};
	return b2Vec3(a.m_float4 * s4);
#endif
}

/// Add two vectors component-wise.
inline b2Vec3 operator + (const b2Vec3& a, const b2Vec3& b)
{
#if !defined(SIMD)
	return b2Vec3(a.m_x + b.m_x, a.m_y + b.m_y, a.m_z + b.m_z);
#else
	return b2Vec3(a.m_float4 + b.m_float4);
#endif
}

/// Subtract two vectors component-wise.
inline b2Vec3 operator - (const b2Vec3& a, const b2Vec3& b)
{
#if !defined(SIMD)
	return b2Vec3(a.m_x - b.m_x, a.m_y - b.m_y, a.m_z - b.m_z);
#else
	return b2Vec3(a.m_float4 - b.m_float4);
#endif
}

/// Perform the dot product on two vectors.
inline float32 b2Dot(const b2Vec3& a, const b2Vec3& b)
{
#if !defined(SIMD)
	return a.m_x * b.m_x + a.m_y * b.m_y + a.m_z * b.m_z;
#else
	float32x4 f4 = a.m_float4 * b.m_float4;
	return f4[0] + f4[1] + f4[2];
#endif
}

/// Perform the cross product on two vectors.
inline b2Vec3 b2Cross(const b2Vec3& a, const b2Vec3& b)
{
#if !defined(SIMD)
	return b2Vec3(a.m_y * b.m_z - a.m_z * b.m_y, 
	              a.m_z * b.m_x - a.m_x * b.m_z, 
	              a.m_x * b.m_y - a.m_y * b.m_x);
#else
	float32x4 ayzx = __builtin_shufflevector(a.m_float4, a.m_float4, 1, 2, 0, -1);
	float32x4 bzxy = __builtin_shufflevector(b.m_float4, b.m_float4, 2, 0, 1, -1);
	float32x4 f4 = ayzx * bzxy;
	float32x4 azxy = __builtin_shufflevector(a.m_float4, a.m_float4, 2, 0, 1, -1);
	float32x4 byzx = __builtin_shufflevector(b.m_float4, b.m_float4, 1, 2, 0, -1);
	f4 -= azxy * byzx;
	return b2Vec3(f4);
#endif
}

inline b2Mat22 operator + (const b2Mat22& A, const b2Mat22& B)
{
	return b2Mat22(A.ex + B.ex, A.ey + B.ey);
}

// A * B
inline b2Mat22 b2Mul(const b2Mat22& A, const b2Mat22& B)
{
	return b2Mat22(b2Mul(A, B.ex), b2Mul(A, B.ey));
}

// A^T * B
inline b2Mat22 b2MulT(const b2Mat22& A, const b2Mat22& B)
{
	b2Vec2 c1(b2Dot(A.ex, B.ex), b2Dot(A.ey, B.ex));
	b2Vec2 c2(b2Dot(A.ex, B.ey), b2Dot(A.ey, B.ey));
	return b2Mat22(c1, c2);
}

/// Multiply a matrix times a vector.
inline b2Vec3 b2Mul(const b2Mat33& A, const b2Vec3& v)
{
	return v.x() * A.ex + v.y() * A.ey + v.z() * A.ez;
	/*
	float32x4 f4 = A.ex.m_float4 * __builtin_shufflevector(v.m_float4, v.m_float4, 0, 0, 0, -1);
	f4 += A.ey.m_float4 * __builtin_shufflevector(v.m_float4, v.m_float4, 1, 1, 1, -1);
	f4 += A.ez.m_float4 * __builtin_shufflevector(v.m_float4, v.m_float4, 2, 2, 2, -1);
	return b2Vec3(f4);
	*/
}

/// Multiply a matrix times a vector.
inline b2Vec2 b2Mul22(const b2Mat33& A, const b2Vec2& v)
{
#if !defined(SIMD)
	return b2Vec2(A.ex.m_x * v.m_x + A.ey.m_x * v.m_y,
	              A.ex.m_y * v.m_x + A.ey.m_y * v.m_y);
#else
	float32x4 f4 = A.ex.m_float4 * __builtin_shufflevector(v.m_float4, v.m_float4, 0, 0, -1, -1);
	f4 += A.ey.m_float4 * __builtin_shufflevector(v.m_float4, v.m_float4, 1, 1, -1, -1);
	return b2Vec2(f4);
#endif
}

/// Multiply two rotations: q * r
inline b2Rot b2Mul(const b2Rot& q, const b2Rot& r)
{
	// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
	// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
	// s = qs * rc + qc * rs
	// c = qc * rc - qs * rs
#if !defined(SIMD)
	b2Rot qr;
	qr.m_s = q.m_s * r.m_c + q.m_c * r.m_s;
	qr.m_c = q.m_c * r.m_c - q.m_s * r.m_s;
	return qr;
#else
	/*
	float32x4 r4 = {1.0, -1.0, 0.0, 0.0};
	float32x4 f4 = q.m_float4 * __builtin_shufflevector(r.m_float4, r.m_float4, 1, 1, -1, -1);
	f4 += __builtin_shufflevector(q.m_float4, q.m_float4, 1, 0, -1, -1) * 
	      __builtin_shufflevector(r.m_float4, r.m_float4, 0, 0, -1, -1) *
	      r4;
	return b2Rot(f4);
	*/
	float32x4 s4 = q.m_float4 * __builtin_shufflevector(r.m_float4, r.m_float4, 1, 0, -1, -1);
	float32x4 c4 = q.m_float4 * r.m_float4;
	return b2Rot(s4[0] + s4[1], -c4[0] + c4[1]);
#endif
}

/// Transpose multiply two rotations: qT * r
inline b2Rot b2MulT(const b2Rot& q, const b2Rot& r)
{
	// [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
	// [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
	// s = qc * rs - qs * rc
	// c = qc * rc + qs * rs
#if !defined(SIMD)
	b2Rot qr;
	qr.m_s = q.m_c * r.m_s - q.m_s * r.m_c;
	qr.m_c = q.m_c * r.m_c + q.m_s * r.m_s;
	return qr;
#else
	/*
	float32x4 r4 = {-1.0, 1.0, 0.0, 0.0};
	float32x4 f4 = __builtin_shufflevector(q.m_float4, q.m_float4, 1, 1, -1, -1) * r.m_float4;
	f4 += __builtin_shufflevector(q.m_float4, q.m_float4, 0, 0, -1, -1) *
	      __builtin_shufflevector(r.m_float4, r.m_float4, 1, 1, -1, -1) *
	      r4;
	return b2Rot(f4);
	*/
	float32x4 s4 = __builtin_shufflevector(q.m_float4, q.m_float4, 1, 0, -1, -1) * r.m_float4;
	float32x4 c4 = q.m_float4 * r.m_float4;
	return b2Rot(s4[0] - s4[1], c4[0] + c4[1]);	
#endif
}

/// Rotate a vector
inline b2Vec2 b2Mul(const b2Rot& q, const b2Vec2& v)
{
#if !defined(SIMD)
	return b2Vec2(q.m_c * v.m_x - q.m_s * v.m_y, q.m_s * v.m_x + q.m_c * v.m_y);
#else
	/*
	float32x4 r4 = {-1.0, 1.0, 0.0, 0.0};
	float32x4 f4 = __builtin_shufflevector(q.m_float4, q.m_float4, 1, 0, -1, -1) *
	               __builtin_shufflevector(v.m_float4, v.m_float4, 0, 0, -1, -1);
	f4 += q.m_float4 * __builtin_shufflevector(v.m_float4, v.m_float4, 1, 1, -1, -1) *
	      r4;
	return b2Vec2(f4);
	*/
	float32x4 x4 = __builtin_shufflevector(q.m_float4, q.m_float4, 1, 0, -1, -1) * v.m_float4;
	float32x4 y4 = q.m_float4 * v.m_float4;
	return b2Vec2(x4[0] - x4[1], y4[0] + y4[1]);
#endif
}

/// Inverse rotate a vector
inline b2Vec2 b2MulT(const b2Rot& q, const b2Vec2& v)
{
#if !defined(SIMD)
	return b2Vec2(q.m_c * v.m_x + q.m_s * v.m_y, -q.m_s * v.m_x + q.m_c * v.m_y);
#else
	/*
	float32x4 r4 = {1.0, -1.0, 0.0, 0.0};
	float32x4 f4 = __builtin_shufflevector(q.m_float4, q.m_float4, 1, 0, -1, -1) * 
	               __builtin_shufflevector(v.m_float4, v.m_float4, 0, 0, -1, -1) *
	               r4;
	f4 += q.m_float4 * __builtin_shufflevector(v.m_float4, v.m_float4, 1, 1, -1, -1);
	return b2Vec2(f4);
	*/
	float32x4 x4 = __builtin_shufflevector(q.m_float4, q.m_float4, 1, 0, -1, -1) * v.m_float4;
	float32x4 y4 = q.m_float4 * v.m_float4;
	return b2Vec2(x4[0] + x4[1], -y4[0] + y4[1]);
#endif
}

inline b2Vec2 b2Mul(const b2Transform& T, const b2Vec2& v)
{
#if !defined(SIMD)
	float32 x = (T.q.m_c * v.m_x - T.q.m_s * v.m_y) + T.p.m_x;
	float32 y = (T.q.m_s * v.m_x + T.q.m_c * v.m_y) + T.p.m_y;
	return b2Vec2(x, y);
#else
	/*
	float32x4 f4 = __builtin_shufflevector(T.q.m_float4, T.q.m_float4, 1, 0, -1, -1) * 
	               __builtin_shufflevector(v.m_float4, v.m_float4, 0, 0, -1, -1);
	f4 += T.q.m_float4 * __builtin_shufflevector(v.m_float4, v.m_float4, 1, 1, -1, -1);
	f4 += T.p.m_float4;
	return b2Vec2(f4);
	*/
	float32x4 x4 = __builtin_shufflevector(T.q.m_float4, T.q.m_float4, 1, 0, -1, -1) * v.m_float4;
	float32x4 y4 = T.q.m_float4 * v.m_float4;
	float32x4 r4 = {x4[0] - x4[1], y4[0] + y4[1], 0.0, 0.0};
	r4 += T.p.m_float4;
	return b2Vec2(r4);
#endif
}

inline b2Vec2 b2MulT(const b2Transform& T, const b2Vec2& v)
{
#if !defined(SIMD)
	float32 px = v.m_x - T.p.m_x;
	float32 py = v.m_y - T.p.m_y;
	float32 x = (T.q.m_c * px + T.q.m_s * py);
	float32 y = (-T.q.m_s * px + T.q.m_c * py);
	return b2Vec2(x, y);
#else
	/*
	float32x4 r4 = {1.0, -1.0, 0.0, 0.0};
	float32x4 p4 = v.m_float4 - T.p.m_float4;
	float32x4 f4 = __builtin_shufflevector(T.q.m_float4, T.q.m_float4, 1, 0, -1, -1) *
	               r4 * __builtin_shufflevector(p4, p4, 0, 0, -1, -1);
	f4 += T.q.m_float4 * __builtin_shufflevector(p4, p4, 1, 1, -1, -1);
	*/
	float32 px = v.x() - T.p.x();
	float32 py = v.y() - T.p.y();
	float32x4 p4 = {px, py, 0.0, 0.0};
	float32x4 x4 = __builtin_shufflevector(T.q.m_float4, T.q.m_float4, 1, 0, -1, -1) * p4;
	float32x4 y4 = T.q.m_float4 * p4;
	return b2Vec2(x4[0] + x4[1], -y4[0] + y4[1]);
#endif
}

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
inline b2Transform b2Mul(const b2Transform& A, const b2Transform& B)
{
	b2Transform C;
	C.q = b2Mul(A.q, B.q);
	C.p = b2Mul(A.q, B.p) + A.p;
	return C;
}

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
inline b2Transform b2MulT(const b2Transform& A, const b2Transform& B)
{
	b2Transform C;
	C.q = b2MulT(A.q, B.q);
	C.p = b2MulT(A.q, B.p - A.p);
	return C;
}

template <typename T>
inline T b2Abs(T a)
{
	return a > T(0) ? a : -a;
}

inline b2Vec2 b2Abs(const b2Vec2& a)
{
	return b2Vec2(b2Abs(a.x()), b2Abs(a.y()));
}

inline b2Mat22 b2Abs(const b2Mat22& A)
{
	return b2Mat22(b2Abs(A.ex), b2Abs(A.ey));
}

template <typename T>
inline T b2Min(T a, T b)
{
	return a < b ? a : b;
}

inline b2Vec2 b2Min(const b2Vec2& a, const b2Vec2& b)
{
	return b2Vec2(b2Min(a.x(), b.x()), b2Min(a.y(), b.y()));
}

template <typename T>
inline T b2Max(T a, T b)
{
	return a > b ? a : b;
}

inline b2Vec2 b2Max(const b2Vec2& a, const b2Vec2& b)
{
	return b2Vec2(b2Max(a.x(), b.x()), b2Max(a.y(), b.y()));
}

template <typename T>
inline T b2Clamp(T a, T low, T high)
{
	return b2Max(low, b2Min(a, high));
}

inline b2Vec2 b2Clamp(const b2Vec2& a, const b2Vec2& low, const b2Vec2& high)
{
	return b2Max(low, b2Min(a, high));
}

template<typename T> inline void b2Swap(T& a, T& b)
{
	T tmp = a;
	a = b;
	b = tmp;
}

/// "Next Largest Power of 2
/// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
/// largest power of 2. For a 32-bit value:"
inline uint32 b2NextPowerOfTwo(uint32 x)
{
	x |= (x >> 1);
	x |= (x >> 2);
	x |= (x >> 4);
	x |= (x >> 8);
	x |= (x >> 16);
	return x + 1;
}

inline bool b2IsPowerOfTwo(uint32 x)
{
	bool result = x > 0 && (x & (x - 1)) == 0;
	return result;
}

inline void b2Sweep::GetTransform(b2Transform* xf, float32 beta) const
{
	xf->p = (1.0f - beta) * c0 + beta * c;
	float32 angle = (1.0f - beta) * a0 + beta * a;
	xf->q.Set(angle);

	// Shift to origin
	xf->p -= b2Mul(xf->q, localCenter);
}

inline void b2Sweep::Advance(float32 alpha)
{
	b2Assert(alpha0 < 1.0f);
	float32 beta = (alpha - alpha0) / (1.0f - alpha0);
	c0 = (1.0f - beta) * c0 + beta * c;
	a0 = (1.0f - beta) * a0 + beta * a;
	alpha0 = alpha;
}

/// Normalize an angle in radians to be between -pi and pi
inline void b2Sweep::Normalize()
{
	float32 twoPi = 2.0f * b2_pi;
	float32 d =  twoPi * floorf(a0 / twoPi);
	a0 -= d;
	a -= d;
}

#endif
