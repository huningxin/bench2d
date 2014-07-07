#include <cstdio>
#include <time.h>
#include <math.h>
#include <stdlib.h>

#include "Box2D/Box2D.h"

void b2Vec2Test() {
	printf("=== b2Vec2Test ===\n");
	// b2Vec2
	b2Vec2 a(1.0, 2.0);
	b2Vec2 b(3.0, 4.0);

	printf("a.x() = %f, a.y() = %f\n", a.x(), a.y());
	printf("b.x() = %f, b.y() = %f\n", b.x(), b.y());

	a += b;
	printf("a.x() = %f, a.y() = %f\n", a.x(), a.y());
	
	a -= b;
	printf("a.x() = %f, a.y() = %f\n", a.x(), a.y());

	a *= 5.0;
	printf("a.x() = %f, a.y() = %f\n", a.x(), a.y());

	a += -b;
	printf("a.x() = %f, a.y() = %f\n", a.x(), a.y());

	printf("a.Length() = %f\n", a.Length());
	printf("a.x() = %f, a.y() = %f\n", a.x(), a.y());
	printf("a.LengthSquared() = %f\n", a.LengthSquared());
	printf("a.x() = %f, a.y() = %f\n", a.x(), a.y());
	printf("a.Normalize() = %f\n", a.Normalize());
	printf("a.x() = %f, a.y() = %f\n", a.x(), a.y());
	printf("a.IsValid() = %d\n", a.IsValid());
	printf("a.x() = %f, a.y() = %f\n", a.x(), a.y());
}

void b2Vec3Test() {
	printf("=== b2Vec3Test ===\n");
	// b2Vec3
	b2Vec3 a(1.0, 2.0, 3.0);
	b2Vec3 b(4.0, 5.0, 6.0);

    printf("a.x() = %f, a.y() = %f, a.z() = %f\n", a.x(), a.y(), a.z());
	printf("b.x() = %f, b.y() = %f, b.z() = %f\n", b.x(), b.y(), b.z());

	a += b;
	printf("a.x() = %f, a.y() = %f, a.z() = %f\n", a.x(), a.y(), a.z());

	a -= b;
	printf("a.x() = %f, a.y() = %f, a.z() = %f\n", a.x(), a.y(), a.z());

	a *= 5.0;
	printf("a.x() = %f, a.y() = %f, a.z() = %f\n", a.x(), a.y(), a.z());

	a += -b;
	printf("a.x() = %f, a.y() = %f, a.z() = %f\n", a.x(), a.y(), a.z());
}

void b2FunctionTest() {
	printf("=== b2FunctionTest ===\n");
	b2Vec2 a(1.1, 2.2);
	b2Vec2 b(3.3, 4.4);
	b2Vec2 c(5.5, 6.6);
	b2Vec2 d(7.7, 8.8);

	float32 r = b2Dot(a, b);
	printf("r = %f\n", r);

	r = b2Cross(a, b);
	printf("r = %f\n", r);

	b2Vec2 v = b2Cross(a, r);
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	v = b2Cross(r, a);
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	b2Mat22 m(a, b);
	v = b2Mul(m, c);
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	v = b2MulT(m, c);
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	v = a + b;
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	v = a - b;
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	v = 5.5 * a;
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	bool e = a == b;
	printf("e = %d\n", e);

	e = b == b;
	printf("e = %d\n", e);

	r = b2Distance(a, b);
	printf("r = %f\n", r);

	r = b2DistanceSquared(a, b);
	printf("r = %f\n", r);

	b2Vec3 a3(1.1, 2.2, 3.3);
	b2Vec3 b3(4.4, 5.5, 6.6);
	b2Vec3 c3(7.7, 8.8, 9.9);

	b2Vec3 v3 = 7.7 * a3;
	printf("v3.x() = %f, v3.y() = %f, v3.z() = %f\n", v3.x(), v3.y(), v3.z());

	v3 = a3 + b3;
	printf("v3.x() = %f, v3.y() = %f, v3.z() = %f\n", v3.x(), v3.y(), v3.z());

	v3 = a3 - b3;
	printf("v3.x() = %f, v3.y() = %f, v3.z() = %f\n", v3.x(), v3.y(), v3.z());

	r = b2Dot(a3, b3);
	printf("r = %f\n", r);

	v3 = b2Cross(a3, b3);
	printf("v3.x() = %f, v3.y() = %f, v3.z() = %f\n", v3.x(), v3.y(), v3.z());

	b2Mat22 A(a, b);
	b2Mat22 B(c, d);

	b2Mat22 C = A + B;
	printf("C.ex.x() = %f, C.ex.y() = %f, C.ey.x() = %f, C.ey.y() = %f\n", C.ex.x(), C.ex.y(), C.ey.x(), C.ey.y());

	C = b2Mul(A, B);		
	printf("C.ex.x() = %f, C.ex.y() = %f, C.ey.x() = %f, C.ey.y() = %f\n", C.ex.x(), C.ex.y(), C.ey.x(), C.ey.y());

	C = b2MulT(A, B);
	printf("C.ex.x() = %f, C.ex.y() = %f, C.ey.x() = %f, C.ey.y() = %f\n", C.ex.x(), C.ex.y(), C.ey.x(), C.ey.y());

	b2Mat33 A3(a3, b3, c3);
	v3 = b2Mul(A3, c3);
	printf("v3.x() = %f, v3.y() = %f, v3.z() = %f\n", v3.x(), v3.y(), v3.z());

	v = b2Mul22(A3, a);
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	b2Rot q(30.0), o(60.0);
	b2Rot p = b2Mul(q, o);
	printf("p.s() = %f, p.c() = %f\n", p.s(), p.c());

	p = b2MulT(q, o);
	printf("p.s() = %f, p.c() = %f\n", p.s(), p.c());

	v = b2Mul(q, b);
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	v = b2MulT(q, b);
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	b2Transform t(c, q);
	v = b2Mul(t, a);
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	v = b2MulT(t, a);
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	b2Transform s(b, o);
	b2Transform w = b2Mul(t, s);
	printf("w.p.x() = %f, w.p.y() = %f, w.q.s() = %f, w.q.c() = %f\n", w.p.x(), w.p.y(), w.q.s(), w.q.c());

	w = b2MulT(t, s);
	printf("w.p.x() = %f, w.p.y() = %f, w.q.s() = %f, w.q.c() = %f\n", w.p.x(), w.p.y(), w.q.s(), w.q.c());
}

void b2FunctionTest1() {
	printf("=== b2FunctionTest1 ===\n");
	b2Vec2 a(-1.1, 2.2);
	b2Vec2 b(3.3, -4.4);

	b2Vec2 v = b2Abs(a);
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	v = b2Min(a, b);
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	v = b2Max(a, b);
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	v = b2Clamp(a, b2Vec2(-2.0, -2.0), b2Vec2(3.0, 3.0));
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());
}

void b2FunctionTest2() {
	printf("=== b2FunctionTest2 ===\n");
	b2Vec2 v(-1.1, 2.2);

	printf("v.x() = %f, -v.y() = %f\n", v.x(), -v.y());
	printf("-v.x() = %f, v.y() = %f\n", -v.x(), v.y());		
}

void b2FunctionTest3 () {
	printf("=== b2FunctionTest3 ===\n");
	b2Vec2 v;

	v.set_x(1.1);
	v.set_y(2.2);
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	v.SetZero();
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	v.Set(3.3, 4.4);
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	v = -v;
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());

	b2Vec3 v3;
	v3.set_x(1.1);
	v3.set_y(2.2);
	v3.set_z(3.3);
	printf("v3.x() = %f, v3.y() = %f, v3.z() = %f\n", v3.x(), v3.y(), v3.z());

	v3.SetZero();
	printf("v3.x() = %f, v3.y() = %f, v3.z() = %f\n", v3.x(), v3.y(), v3.z());

	v3.Set(4.4, 5.5, 6.6);
	printf("v3.x() = %f, v3.y() = %f, v3.z() = %f\n", v3.x(), v3.y(), v3.z());

	v3 = -v3;
	printf("v3.x() = %f, v3.y() = %f, v3.z() = %f\n", v3.x(), v3.y(), v3.z());

	b2Mat22 C(1.1, 2.2, 3.3, 4.4);
	printf("C.ex.x() = %f, C.ex.y() = %f, C.ey.x() = %f, C.ey.y() = %f\n", C.ex.x(), C.ex.y(), C.ey.x(), C.ey.y());	

	C.SetIdentity();
	printf("C.ex.x() = %f, C.ex.y() = %f, C.ey.x() = %f, C.ey.y() = %f\n", C.ex.x(), C.ex.y(), C.ey.x(), C.ey.y());

	C.SetZero();
	printf("C.ex.x() = %f, C.ex.y() = %f, C.ey.x() = %f, C.ey.y() = %f\n", C.ex.x(), C.ex.y(), C.ey.x(), C.ey.y());

	C.Set(v, v);
	printf("C.ex.x() = %f, C.ex.y() = %f, C.ey.x() = %f, C.ey.y() = %f\n", C.ex.x(), C.ex.y(), C.ey.x(), C.ey.y());

	C = C.GetInverse();
	printf("C.ex.x() = %f, C.ex.y() = %f, C.ey.x() = %f, C.ey.y() = %f\n", C.ex.x(), C.ex.y(), C.ey.x(), C.ey.y());

	v = C.Solve(v);
	printf("v.x() = %f, v.y() = %f\n", v.x(), v.y());


}

void b2MathTest() {
	b2Vec2Test();
	b2Vec3Test();
	b2FunctionTest();
	b2FunctionTest1();
	b2FunctionTest2();
	b2FunctionTest3();
}