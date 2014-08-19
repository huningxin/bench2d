/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <xmmintrin.h>

#include <Box2D/Dynamics/Contacts/b2ContactSolver.h>

#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Common/b2StackAllocator.h>

#define B2_DEBUG_SOLVER 0
#define B2_DEBUG_SIMD 0

struct b2ContactPositionConstraint
{
	b2Vec2 localPoints[b2_maxManifoldPoints];
	b2Vec2 localNormal;
	b2Vec2 localPoint;
	int32 indexA;
	int32 indexB;
	float32 invMassA, invMassB;
	b2Vec2 localCenterA, localCenterB;
	float32 invIA, invIB;
	b2Manifold::Type type;
	float32 radiusA, radiusB;
	int32 pointCount;
};

b2ContactSolver::b2ContactSolver(b2ContactSolverDef* def)
{
	m_step = def->step;
	m_allocator = def->allocator;
	m_count = def->count;
	m_positionConstraints = (b2ContactPositionConstraint*)m_allocator->Allocate(m_count * sizeof(b2ContactPositionConstraint));
	m_velocityConstraints = (b2ContactVelocityConstraint*)m_allocator->Allocate(m_count * sizeof(b2ContactVelocityConstraint));
	m_positions = def->positions;
	m_velocities = def->velocities;
	m_contacts = def->contacts;

	// Initialize position independent portions of the constraints.
	for (int32 i = 0; i < m_count; ++i)
	{
		b2Contact* contact = m_contacts[i];

		b2Fixture* fixtureA = contact->m_fixtureA;
		b2Fixture* fixtureB = contact->m_fixtureB;
		b2Shape* shapeA = fixtureA->GetShape();
		b2Shape* shapeB = fixtureB->GetShape();
		float32 radiusA = shapeA->m_radius;
		float32 radiusB = shapeB->m_radius;
		b2Body* bodyA = fixtureA->GetBody();
		b2Body* bodyB = fixtureB->GetBody();
		b2Manifold* manifold = contact->GetManifold();

		int32 pointCount = manifold->pointCount;
		b2Assert(pointCount > 0);

		b2ContactVelocityConstraint* vc = m_velocityConstraints + i;
		vc->friction = contact->m_friction;
		vc->restitution = contact->m_restitution;
		vc->indexA = bodyA->m_islandIndex;
		vc->indexB = bodyB->m_islandIndex;
		vc->invMassA = bodyA->m_invMass;
		vc->invMassB = bodyB->m_invMass;
		vc->invIA = bodyA->m_invI;
		vc->invIB = bodyB->m_invI;
		vc->contactIndex = i;
		vc->pointCount = pointCount;
		vc->K.SetZero();
		vc->normalMass.SetZero();

		b2ContactPositionConstraint* pc = m_positionConstraints + i;
		pc->indexA = bodyA->m_islandIndex;
		pc->indexB = bodyB->m_islandIndex;
		pc->invMassA = bodyA->m_invMass;
		pc->invMassB = bodyB->m_invMass;
		pc->localCenterA = bodyA->m_sweep.localCenter;
		pc->localCenterB = bodyB->m_sweep.localCenter;
		pc->invIA = bodyA->m_invI;
		pc->invIB = bodyB->m_invI;
		pc->localNormal = manifold->localNormal;
		pc->localPoint = manifold->localPoint;
		pc->pointCount = pointCount;
		pc->radiusA = radiusA;
		pc->radiusB = radiusB;
		pc->type = manifold->type;

		for (int32 j = 0; j < pointCount; ++j)
		{
			b2ManifoldPoint* cp = manifold->points + j;
			b2VelocityConstraintPoint* vcp = vc->points + j;
	
			if (m_step.warmStarting)
			{
				vcp->normalImpulse = m_step.dtRatio * cp->normalImpulse;
				vcp->tangentImpulse = m_step.dtRatio * cp->tangentImpulse;
			}
			else
			{
				vcp->normalImpulse = 0.0f;
				vcp->tangentImpulse = 0.0f;
			}

			vcp->rA.SetZero();
			vcp->rB.SetZero();
			vcp->normalMass = 0.0f;
			vcp->tangentMass = 0.0f;
			vcp->velocityBias = 0.0f;

			pc->localPoints[j] = cp->localPoint;
		}
	}
}

b2ContactSolver::~b2ContactSolver()
{
	m_allocator->Free(m_velocityConstraints);
	m_allocator->Free(m_positionConstraints);
}

// Initialize position dependent portions of the velocity constraints.
void b2ContactSolver::InitializeVelocityConstraints()
{
	for (int32 i = 0; i < m_count; ++i)
	{
		b2ContactVelocityConstraint* vc = m_velocityConstraints + i;
		b2ContactPositionConstraint* pc = m_positionConstraints + i;

		float32 radiusA = pc->radiusA;
		float32 radiusB = pc->radiusB;
		b2Manifold* manifold = m_contacts[vc->contactIndex]->GetManifold();

		int32 indexA = vc->indexA;
		int32 indexB = vc->indexB;

		float32 mA = vc->invMassA;
		float32 mB = vc->invMassB;
		float32 iA = vc->invIA;
		float32 iB = vc->invIB;
		b2Vec2 localCenterA = pc->localCenterA;
		b2Vec2 localCenterB = pc->localCenterB;

		b2Vec2 cA = m_positions[indexA].c;
		float32 aA = m_positions[indexA].a;
		b2Vec2 vA = m_velocities[indexA].v;
		float32 wA = m_velocities[indexA].w;

		b2Vec2 cB = m_positions[indexB].c;
		float32 aB = m_positions[indexB].a;
		b2Vec2 vB = m_velocities[indexB].v;
		float32 wB = m_velocities[indexB].w;

		b2Assert(manifold->pointCount > 0);

		b2Transform xfA, xfB;
		xfA.q.Set(aA);
		xfB.q.Set(aB);
		xfA.p = cA - b2Mul(xfA.q, localCenterA);
		xfB.p = cB - b2Mul(xfB.q, localCenterB);

		b2WorldManifold worldManifold;
		worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB);

		vc->normal = worldManifold.normal;

		int32 pointCount = vc->pointCount;
		for (int32 j = 0; j < pointCount; ++j)
		{
			b2VelocityConstraintPoint* vcp = vc->points + j;

			vcp->rA = worldManifold.points[j] - cA;
			vcp->rB = worldManifold.points[j] - cB;

			float32 rnA = b2Cross(vcp->rA, vc->normal);
			float32 rnB = b2Cross(vcp->rB, vc->normal);

			float32 kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			vcp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

			b2Vec2 tangent = b2Cross(vc->normal, 1.0f);

			float32 rtA = b2Cross(vcp->rA, tangent);
			float32 rtB = b2Cross(vcp->rB, tangent);

			float32 kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

			vcp->tangentMass = kTangent > 0.0f ? 1.0f /  kTangent : 0.0f;

			// Setup a velocity bias for restitution.
			vcp->velocityBias = 0.0f;
			float32 vRel = b2Dot(vc->normal, vB + b2Cross(wB, vcp->rB) - vA - b2Cross(wA, vcp->rA));
			if (vRel < -b2_velocityThreshold)
			{
				vcp->velocityBias = -vc->restitution * vRel;
			}
		}

		// If we have two points, then prepare the block solver.
		if (vc->pointCount == 2)
		{
			b2VelocityConstraintPoint* vcp1 = vc->points + 0;
			b2VelocityConstraintPoint* vcp2 = vc->points + 1;

			float32 rn1A = b2Cross(vcp1->rA, vc->normal);
			float32 rn1B = b2Cross(vcp1->rB, vc->normal);
			float32 rn2A = b2Cross(vcp2->rA, vc->normal);
			float32 rn2B = b2Cross(vcp2->rB, vc->normal);

			float32 k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
			float32 k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
			float32 k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

			// Ensure a reasonable condition number.
			const float32 k_maxConditionNumber = 1000.0f;
			if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
			{
				// K is safe to invert.
				vc->K.ex.Set(k11, k12);
				vc->K.ey.Set(k12, k22);
				vc->normalMass = vc->K.GetInverse();
			}
			else
			{
				// The constraints are redundant, just use one.
				// TODO_ERIN use deepest?
				vc->pointCount = 1;
			}
		}
	}
}

void b2ContactSolver::WarmStart()
{
	// Warm start.
	for (int32 i = 0; i < m_count; ++i)
	{
		b2ContactVelocityConstraint* vc = m_velocityConstraints + i;

		int32 indexA = vc->indexA;
		int32 indexB = vc->indexB;
		float32 mA = vc->invMassA;
		float32 iA = vc->invIA;
		float32 mB = vc->invMassB;
		float32 iB = vc->invIB;
		int32 pointCount = vc->pointCount;

		b2Vec2 vA = m_velocities[indexA].v;
		float32 wA = m_velocities[indexA].w;
		b2Vec2 vB = m_velocities[indexB].v;
		float32 wB = m_velocities[indexB].w;

		b2Vec2 normal = vc->normal;
		b2Vec2 tangent = b2Cross(normal, 1.0f);

		for (int32 j = 0; j < pointCount; ++j)
		{
			b2VelocityConstraintPoint* vcp = vc->points + j;
			b2Vec2 P = vcp->normalImpulse * normal + vcp->tangentImpulse * tangent;
			wA -= iA * b2Cross(vcp->rA, P);
			vA -= mA * P;
			wB += iB * b2Cross(vcp->rB, P);
			vB += mB * P;
		}

		m_velocities[indexA].v = vA;
		m_velocities[indexA].w = wA;
		m_velocities[indexB].v = vB;
		m_velocities[indexB].w = wB;
	}
}

void b2ContactSolver::SolveVelocityConstraints()
{
	for (int32 i = 0; i < m_count; ++i)
	{
		b2ContactVelocityConstraint* vc = m_velocityConstraints + i;

		int32 indexA = vc->indexA;
		int32 indexB = vc->indexB;
		float32 mA = vc->invMassA;
		float32 iA = vc->invIA;
		float32 mB = vc->invMassB;
		float32 iB = vc->invIB;
		int32 pointCount = vc->pointCount;

		b2Vec2 vA = m_velocities[indexA].v;
		float32 wA = m_velocities[indexA].w;
		b2Vec2 vB = m_velocities[indexB].v;
		float32 wB = m_velocities[indexB].w;

		b2Vec2 normal = vc->normal;
		b2Vec2 tangent = b2Cross(normal, 1.0f);
		float32 friction = vc->friction;

		b2Assert(pointCount == 1 || pointCount == 2);

		// Solve tangent constraints first because non-penetration is more important
		// than friction.
		for (int32 j = 0; j < pointCount; ++j)
		{
			b2VelocityConstraintPoint* vcp = vc->points + j;

			// Relative velocity at contact
			b2Vec2 dv = vB + b2Cross(wB, vcp->rB) - vA - b2Cross(wA, vcp->rA);

			// Compute tangent force
			float32 vt = b2Dot(dv, tangent);
			float32 lambda = vcp->tangentMass * (-vt);

			// b2Clamp the accumulated force
			float32 maxFriction = friction * vcp->normalImpulse;
			float32 newImpulse = b2Clamp(vcp->tangentImpulse + lambda, -maxFriction, maxFriction);
			lambda = newImpulse - vcp->tangentImpulse;
			vcp->tangentImpulse = newImpulse;

			// Apply contact impulse
			b2Vec2 P = lambda * tangent;

			vA -= mA * P;
			wA -= iA * b2Cross(vcp->rA, P);

			vB += mB * P;
			wB += iB * b2Cross(vcp->rB, P);
		}

		// Solve normal constraints
		if (vc->pointCount == 1)
		{
			b2VelocityConstraintPoint* vcp = vc->points + 0;

			// Relative velocity at contact
			b2Vec2 dv = vB + b2Cross(wB, vcp->rB) - vA - b2Cross(wA, vcp->rA);

			// Compute normal impulse
			float32 vn = b2Dot(dv, normal);
			float32 lambda = -vcp->normalMass * (vn - vcp->velocityBias);

			// b2Clamp the accumulated impulse
			float32 newImpulse = b2Max(vcp->normalImpulse + lambda, 0.0f);
			lambda = newImpulse - vcp->normalImpulse;
			vcp->normalImpulse = newImpulse;

			// Apply contact impulse
			b2Vec2 P = lambda * normal;
			vA -= mA * P;
			wA -= iA * b2Cross(vcp->rA, P);

			vB += mB * P;
			wB += iB * b2Cross(vcp->rB, P);
		}
		else
		{
			// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
			// Build the mini LCP for this contact patch
			//
			// vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
			//
			// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
			// b = vn0 - velocityBias
			//
			// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
			// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
			// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
			// solution that satisfies the problem is chosen.
			// 
			// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
			// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
			//
			// Substitute:
			// 
			// x = a + d
			// 
			// a := old total impulse
			// x := new total impulse
			// d := incremental impulse 
			//
			// For the current iteration we extend the formula for the incremental impulse
			// to compute the new total impulse:
			//
			// vn = A * d + b
			//    = A * (x - a) + b
			//    = A * x + b - A * a
			//    = A * x + b'
			// b' = b - A * a;

			b2VelocityConstraintPoint* cp1 = vc->points + 0;
			b2VelocityConstraintPoint* cp2 = vc->points + 1;

			b2Vec2 a(cp1->normalImpulse, cp2->normalImpulse);
			b2Assert(a.x >= 0.0f && a.y >= 0.0f);

			// Relative velocity at contact
			b2Vec2 dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
			b2Vec2 dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);

			// Compute normal velocity
			float32 vn1 = b2Dot(dv1, normal);
			float32 vn2 = b2Dot(dv2, normal);

			b2Vec2 b;
			b.x = vn1 - cp1->velocityBias;
			b.y = vn2 - cp2->velocityBias;

			// Compute b'
			b -= b2Mul(vc->K, a);

			const float32 k_errorTol = 1e-3f;
			B2_NOT_USED(k_errorTol);

			for (;;)
			{
				//
				// Case 1: vn = 0
				//
				// 0 = A * x + b'
				//
				// Solve for x:
				//
				// x = - inv(A) * b'
				//
				b2Vec2 x = - b2Mul(vc->normalMass, b);

				if (x.x >= 0.0f && x.y >= 0.0f)
				{
					// Get the incremental impulse
					b2Vec2 d = x - a;

					// Apply incremental impulse
					b2Vec2 P1 = d.x * normal;
					b2Vec2 P2 = d.y * normal;
					vA -= mA * (P1 + P2);
					wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));

					vB += mB * (P1 + P2);
					wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

#if B2_DEBUG_SOLVER == 1
					// Postconditions
					dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
					dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);

					// Compute normal velocity
					vn1 = b2Dot(dv1, normal);
					vn2 = b2Dot(dv2, normal);

					b2Assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
					b2Assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
#endif
					break;
				}

				//
				// Case 2: vn1 = 0 and x2 = 0
				//
				//   0 = a11 * x1 + a12 * 0 + b1' 
				// vn2 = a21 * x1 + a22 * 0 + b2'
				//
				x.x = - cp1->normalMass * b.x;
				x.y = 0.0f;
				vn1 = 0.0f;
				vn2 = vc->K.ex.y * x.x + b.y;

				if (x.x >= 0.0f && vn2 >= 0.0f)
				{
					// Get the incremental impulse
					b2Vec2 d = x - a;

					// Apply incremental impulse
					b2Vec2 P1 = d.x * normal;
					b2Vec2 P2 = d.y * normal;
					vA -= mA * (P1 + P2);
					wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));

					vB += mB * (P1 + P2);
					wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

#if B2_DEBUG_SOLVER == 1
					// Postconditions
					dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);

					// Compute normal velocity
					vn1 = b2Dot(dv1, normal);

					b2Assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
#endif
					break;
				}


				//
				// Case 3: vn2 = 0 and x1 = 0
				//
				// vn1 = a11 * 0 + a12 * x2 + b1' 
				//   0 = a21 * 0 + a22 * x2 + b2'
				//
				x.x = 0.0f;
				x.y = - cp2->normalMass * b.y;
				vn1 = vc->K.ey.x * x.y + b.x;
				vn2 = 0.0f;

				if (x.y >= 0.0f && vn1 >= 0.0f)
				{
					// Resubstitute for the incremental impulse
					b2Vec2 d = x - a;

					// Apply incremental impulse
					b2Vec2 P1 = d.x * normal;
					b2Vec2 P2 = d.y * normal;
					vA -= mA * (P1 + P2);
					wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));

					vB += mB * (P1 + P2);
					wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

#if B2_DEBUG_SOLVER == 1
					// Postconditions
					dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);

					// Compute normal velocity
					vn2 = b2Dot(dv2, normal);

					b2Assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
#endif
					break;
				}

				//
				// Case 4: x1 = 0 and x2 = 0
				// 
				// vn1 = b1
				// vn2 = b2;
				x.x = 0.0f;
				x.y = 0.0f;
				vn1 = b.x;
				vn2 = b.y;

				if (vn1 >= 0.0f && vn2 >= 0.0f )
				{
					// Resubstitute for the incremental impulse
					b2Vec2 d = x - a;

					// Apply incremental impulse
					b2Vec2 P1 = d.x * normal;
					b2Vec2 P2 = d.y * normal;
					vA -= mA * (P1 + P2);
					wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));

					vB += mB * (P1 + P2);
					wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

					break;
				}

				// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
				break;
			}
		}

		m_velocities[indexA].v = vA;
		m_velocities[indexA].w = wA;
		m_velocities[indexB].v = vB;
		m_velocities[indexB].w = wB;
	}
}

void b2ContactSolver::StoreImpulses()
{
	for (int32 i = 0; i < m_count; ++i)
	{
		b2ContactVelocityConstraint* vc = m_velocityConstraints + i;
		b2Manifold* manifold = m_contacts[vc->contactIndex]->GetManifold();

		for (int32 j = 0; j < vc->pointCount; ++j)
		{
			manifold->points[j].normalImpulse = vc->points[j].normalImpulse;
			manifold->points[j].tangentImpulse = vc->points[j].tangentImpulse;
		}
	}
}

struct b2PositionSolverManifold
{
	void Initialize(b2ContactPositionConstraint* pc, const b2Transform& xfA, const b2Transform& xfB, int32 index)
	{
		b2Assert(pc->pointCount > 0);

		switch (pc->type)
		{
		case b2Manifold::e_circles:
			{
				b2Vec2 pointA = b2Mul(xfA, pc->localPoint);
				b2Vec2 pointB = b2Mul(xfB, pc->localPoints[0]);
				normal = pointB - pointA;
				normal.Normalize();
				point = 0.5f * (pointA + pointB);
				separation = b2Dot(pointB - pointA, normal) - pc->radiusA - pc->radiusB;
			}
			break;

		case b2Manifold::e_faceA:
			{
				normal = b2Mul(xfA.q, pc->localNormal);
				b2Vec2 planePoint = b2Mul(xfA, pc->localPoint);

				b2Vec2 clipPoint = b2Mul(xfB, pc->localPoints[index]);
				separation = b2Dot(clipPoint - planePoint, normal) - pc->radiusA - pc->radiusB;
				point = clipPoint;
			}
			break;

		case b2Manifold::e_faceB:
			{
				normal = b2Mul(xfB.q, pc->localNormal);
				b2Vec2 planePoint = b2Mul(xfB, pc->localPoint);

				b2Vec2 clipPoint = b2Mul(xfA, pc->localPoints[index]);
				separation = b2Dot(clipPoint - planePoint, normal) - pc->radiusA - pc->radiusB;
				point = clipPoint;

				// Ensure normal points from A to B
				normal = -normal;
			}
			break;
		}
	}

	b2Vec2 normal;
	b2Vec2 point;
	float32 separation;
};

int32 FaceACount1 = 0;
int32 FaceBCount1 = 0;
int32 FaceACount2 = 0;
int32 FaceBCount2 = 0;

float32 b2ContactSolver::SolvePositionConstraintsScalar(b2ContactPositionConstraint** array, int32 count)
{
	float32 minSeparation = 0.0f;
	for (int32 i = 0; i < count; ++i)
	{
		b2ContactPositionConstraint* pc = array[i];

		int32 indexA = pc->indexA;
		int32 indexB = pc->indexB;
		b2Vec2 localCenterA = pc->localCenterA;
		float32 mA = pc->invMassA;
		float32 iA = pc->invIA;
		b2Vec2 localCenterB = pc->localCenterB;
		float32 mB = pc->invMassB;
		float32 iB = pc->invIB;
		int32 pointCount = pc->pointCount;

		b2Vec2 cA = m_positions[indexA].c;
		float32 aA = m_positions[indexA].a;

		b2Vec2 cB = m_positions[indexB].c;
		float32 aB = m_positions[indexB].a;

#if B2_DEBUG_SIMD == 1
		if (pc->type == b2Manifold::e_faceA && pc->pointCount == 2) {
			printf("input - indexA = %d indexB = %d cA.x = %f cA.y = %f aA = %f cB.x = %f cB.y = %f aB = %f\n", indexA, indexB, cA.x, cA.y, aA, cB.x, cB.y, aB);
		}
#endif

		// Solve normal constraints
		for (int32 j = 0; j < pointCount; ++j)
		{
			b2Transform xfA, xfB;
			xfA.q.Set(aA);
			xfB.q.Set(aB);
			xfA.p = cA - b2Mul(xfA.q, localCenterA);
			xfB.p = cB - b2Mul(xfB.q, localCenterB);

#if B2_DEBUG_SIMD == 1
			if (pc->type == b2Manifold::e_faceA && pc->pointCount == 2) {
				printf("xfA.p.x = %f xfA.p.y = %f xfB.p.x = %f xfB.p.y = %f\n", xfA.p.x, xfA.p.y, xfB.p.x, xfB.p.y);
			}
#endif

			b2PositionSolverManifold psm;
			psm.Initialize(pc, xfA, xfB, j);
			b2Vec2 normal = psm.normal;

#if B2_DEBUG_SIMD == 1
			if (pc->type == b2Manifold::e_faceA && pc->pointCount == 2) {
				printf("normal.x = %f normal.y = %f\n", normal.x, normal.y);
			}
#endif

			b2Vec2 point = psm.point;
			float32 separation = psm.separation;

			b2Vec2 rA = point - cA;
			b2Vec2 rB = point - cB;

			// Track max constraint error.
			minSeparation = b2Min(minSeparation, separation);

			// Prevent large corrections and allow slop.
			float32 C = b2Clamp(b2_baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0f);

			// Compute the effective mass.
			float32 rnA = b2Cross(rA, normal);
			float32 rnB = b2Cross(rB, normal);
			float32 K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			// Compute normal impulse
			float32 impulse = K > 0.0f ? - C / K : 0.0f;

			b2Vec2 P = impulse * normal;

#if B2_DEBUG_SIMD == 1
			if (pc->type == b2Manifold::e_faceA && pc->pointCount == 2) {
				printf("P.x = %f P.y = %f\n", P.x, P.y);
			}
#endif

			cA -= mA * P;
			aA -= iA * b2Cross(rA, P);

			cB += mB * P;
			aB += iB * b2Cross(rB, P);
		}

		m_positions[indexA].c = cA;
		m_positions[indexA].a = aA;

		m_positions[indexB].c = cB;
		m_positions[indexB].a = aB;

#if B2_DEBUG_SIMD == 1
		if (pc->type == b2Manifold::e_faceA && pc->pointCount == 2) {
			printf("output - cA.x = %f cA.y = %f aA = %f cB.x = %f cB.y = %f aB = %f\n", cA.x, cA.y, aA, cB.x, cB.y, aB);
		}
#endif
	}

	return minSeparation;
}

float32 b2ContactSolver::SolvePositionConstraintsSIMD(b2ContactPositionConstraint** pc_array, int32 count)
{
	static const __m128 SIGNMASK = _mm_castsi128_ps(_mm_set1_epi32(0x80000000));

	// pc_array contains pcs with same type and pointsCount
	b2Manifold::Type type = pc_array[0]->type;
	int32 pointCount = pc_array[0]->pointCount;

	__m128 minSeparation4 = _mm_set_ps1(0.0f);

	int32 size4 = (int32)(ceilf(count/4.0f)) * 4;

	// Allocate memory.
	float32** localPoints_x_array = (float32**) m_allocator->Allocate(sizeof(float32*) * pointCount);
	float32** localPoints_y_array = (float32**) m_allocator->Allocate(sizeof(float32*) * pointCount);
	float32* memory = (float32*) m_allocator->Allocate((pointCount * 2 + 20) * size4 * sizeof(float32));

	// Set array pointers.
	int32 index = 0;
	for (int32 i = 0; i < pointCount; i++) {
		localPoints_x_array[i] = memory + index++ * size4;
		localPoints_y_array[i] = memory + index++ * size4;
	}
	float32* localNormal_x_array = memory + index++ * size4;
	float32* localNormal_y_array = memory + index++ * size4;
    float32* localPoint_x_array = memory + index++ * size4;
	float32* localPoint_y_array = memory + index++ * size4;
	float32* localCenterA_x_array = memory + index++ * size4;
	float32* localCenterA_y_array = memory + index++ * size4;
	float32* mA_array = memory + index++ * size4;
	float32* iA_array = memory + index++ * size4;
	float32* localCenterB_x_array = memory + index++ * size4;
	float32* localCenterB_y_array = memory + index++ * size4;
	float32* mB_array = memory + index++ * size4;
	float32* iB_array = memory + index++ * size4;
	float32* radiusA_array = memory + index++ * size4;
	float32* radiusB_array = memory + index++ * size4;
	float32* cA_x_array = memory + index++ * size4;
	float32* cA_y_array = memory + index++ * size4;
	float32* aA_array = memory + index++ * size4;
	float32* cB_x_array = memory + index++ * size4;
	float32* cB_y_array = memory + index++ * size4;
	float32* aB_array = memory + index++ * size4;

	// Initialize the arrays.
	for (int32 i = 0; i < count; ++i)
	{
		b2ContactPositionConstraint* pc = pc_array[i];

		for (int32 j = 0; j < pointCount; j++)
		{
			localPoints_x_array[j][i] = pc->localPoints[j].x;
			localPoints_y_array[j][i] = pc->localPoints[j].y;
		}
		localNormal_x_array[i] = pc->localNormal.x;
		localNormal_y_array[i] = pc->localNormal.y;
		localPoint_x_array[i] = pc->localPoint.x;
		localPoint_y_array[i] = pc->localPoint.y;
		localCenterA_x_array[i] = pc->localCenterA.x;
		localCenterA_y_array[i] = pc->localCenterA.y;
		mA_array[i] = pc->invMassA;
		iA_array[i] = pc->invIA;
		localCenterB_x_array[i] = pc->localCenterB.x;
		localCenterB_y_array[i] = pc->localCenterB.y;
		mB_array[i] = pc->invMassB;
		iB_array[i] = pc->invIB;
		radiusA_array[i] = pc->radiusA;
		radiusB_array[i] = pc->radiusB;
	
		int32 indexA = pc->indexA;
		int32 indexB = pc->indexB;
		cA_x_array[i] = m_positions[indexA].c.x;
		cA_y_array[i] = m_positions[indexA].c.y;
		aA_array[i] = m_positions[indexA].a;
		cB_x_array[i] = m_positions[indexB].c.x;
		cB_y_array[i] = m_positions[indexB].c.y;
		aB_array[i] = m_positions[indexB].a;
	}

	for (int32 i = 0; i < size4 - 4; i += 4)
	{
		// SIMD load
		//b2Vec2 localCenterA = pc->localCenterA;
		__m128 localCenterA_x4 = _mm_loadu_ps(localCenterA_x_array + i);
		__m128 localCenterA_y4 = _mm_loadu_ps(localCenterA_y_array + i);
		//float32 mA = pc->invMassA;
		__m128 mA4 = _mm_loadu_ps(mA_array + i);	
		//float32 iA = pc->invIA;
		__m128 iA4 = _mm_loadu_ps(iA_array + i);
		//b2Vec2 localCenterB = pc->localCenterB;
		__m128 localCenterB_x4 = _mm_loadu_ps(localCenterB_x_array + i);
		__m128 localCenterB_y4 = _mm_loadu_ps(localCenterB_y_array + i);
		//float32 mB = pc->invMassB;
		__m128 mB4 = _mm_loadu_ps(mB_array + i);
		//float32 iB = pc->invIB;
		__m128 iB4 = _mm_loadu_ps(iB_array + i);
		//b2Vec2 cA = m_positions[indexA].c;
		__m128 cA_x4 = _mm_loadu_ps(cA_x_array + i);
		__m128 cA_y4 = _mm_loadu_ps(cA_y_array + i);
		//float32 aA = m_positions[indexA].a;
		__m128 aA4 = _mm_loadu_ps(aA_array + i);
		//b2Vec2 cB = m_positions[indexB].c;
		__m128 cB_x4 = _mm_loadu_ps(cB_x_array + i);
		__m128 cB_y4 = _mm_loadu_ps(cB_y_array + i);
		//float32 aB = m_positions[indexB].a;
		__m128 aB4 = _mm_loadu_ps(aB_array + i);

		__m128 localNormal_x4 = _mm_loadu_ps(localNormal_x_array + i);
		__m128 localNormal_y4 = _mm_loadu_ps(localNormal_y_array + i);
		__m128 localPoint_x4 = _mm_loadu_ps(localPoint_x_array + i);
		__m128 localPoint_y4 = _mm_loadu_ps(localPoint_y_array + i);
		__m128 radiusA4 = _mm_loadu_ps(radiusA_array + i);
		__m128 radiusB4 = _mm_loadu_ps(radiusB_array + i);

#if B2_DEBUG_SIMD == 1
		printf("input - cA.x = %f cA.y = %f aA = %f cB.x = %f cB.y = %f aB = %f\n", cA_x4.m128_f32[0], cA_y4.m128_f32[0], aA4.m128_f32[0], cB_x4.m128_f32[0], cB_y4.m128_f32[0], aB4.m128_f32[0]);
		printf("input - cA.x = %f cA.y = %f aA = %f cB.x = %f cB.y = %f aB = %f\n", cA_x4.m128_f32[1], cA_y4.m128_f32[1], aA4.m128_f32[1], cB_x4.m128_f32[1], cB_y4.m128_f32[1], aB4.m128_f32[1]);
		printf("input - cA.x = %f cA.y = %f aA = %f cB.x = %f cB.y = %f aB = %f\n", cA_x4.m128_f32[2], cA_y4.m128_f32[2], aA4.m128_f32[2], cB_x4.m128_f32[2], cB_y4.m128_f32[2], aB4.m128_f32[2]);
		printf("input - cA.x = %f cA.y = %f aA = %f cB.x = %f cB.y = %f aB = %f\n", cA_x4.m128_f32[3], cA_y4.m128_f32[3], aA4.m128_f32[3], cB_x4.m128_f32[3], cB_y4.m128_f32[3], aB4.m128_f32[3]);
#endif

		for (int32 j = 0; j < pointCount; ++j)
		{
			// xfA.q.Set(aA);
			__m128 xfA_q_s4, xfA_q_c4;
			xfA_q_s4.m128_f32[0] = sinf(aA4.m128_f32[0]);
			xfA_q_s4.m128_f32[1] = sinf(aA4.m128_f32[1]);
			xfA_q_s4.m128_f32[2] = sinf(aA4.m128_f32[2]);
			xfA_q_s4.m128_f32[3] = sinf(aA4.m128_f32[3]);
			xfA_q_c4.m128_f32[0] = cosf(aA4.m128_f32[0]);
			xfA_q_c4.m128_f32[1] = cosf(aA4.m128_f32[1]);
			xfA_q_c4.m128_f32[2] = cosf(aA4.m128_f32[2]);
			xfA_q_c4.m128_f32[3] = cosf(aA4.m128_f32[3]);

			// xfB.q.Set(aB);
			__m128 xfB_q_s4, xfB_q_c4;
			xfB_q_s4.m128_f32[0] = sinf(aB4.m128_f32[0]);
			xfB_q_s4.m128_f32[1] = sinf(aB4.m128_f32[1]);
			xfB_q_s4.m128_f32[2] = sinf(aB4.m128_f32[2]);
			xfB_q_s4.m128_f32[3] = sinf(aB4.m128_f32[3]);
			xfB_q_c4.m128_f32[0] = cosf(aB4.m128_f32[0]);
			xfB_q_c4.m128_f32[1] = cosf(aB4.m128_f32[1]);
			xfB_q_c4.m128_f32[2] = cosf(aB4.m128_f32[2]);
			xfB_q_c4.m128_f32[3] = cosf(aB4.m128_f32[3]);

			// xfA.p = cA - b2Mul(xfA.q, localCenterA);
			__m128 xfA_p_x4, xfA_p_y4;
			xfA_p_x4 = _mm_sub_ps(cA_x4, _mm_sub_ps(_mm_mul_ps(xfA_q_c4, localCenterA_x4),
												    _mm_mul_ps(xfA_q_s4, localCenterA_y4)));
			xfA_p_y4 = _mm_sub_ps(cA_y4, _mm_add_ps(_mm_mul_ps(xfA_q_s4, localCenterA_x4),
												    _mm_mul_ps(xfA_q_c4, localCenterA_y4)));

			// xfB.p = cB - b2Mul(xfB.q, localCenterB);
			__m128 xfB_p_x4, xfB_p_y4;
			xfB_p_x4 = _mm_sub_ps(cB_x4, _mm_sub_ps(_mm_mul_ps(xfB_q_c4, localCenterB_x4),
												    _mm_mul_ps(xfB_q_s4, localCenterB_y4)));
			xfB_p_y4 = _mm_sub_ps(cB_y4, _mm_add_ps(_mm_mul_ps(xfB_q_s4, localCenterB_x4),
												    _mm_mul_ps(xfB_q_c4, localCenterB_y4)));

#if B2_DEBUG_SIMD == 1
			printf("xfA.p.x = %f xfA.p.y = %f xfB.p.x = %f xfB.p.y = %f\n", xfA_p_x4.m128_f32[0], xfA_p_y4.m128_f32[0], xfB_p_x4.m128_f32[0], xfB_p_y4.m128_f32[0]);
			printf("xfA.p.x = %f xfA.p.y = %f xfB.p.x = %f xfB.p.y = %f\n", xfA_p_x4.m128_f32[1], xfA_p_y4.m128_f32[1], xfB_p_x4.m128_f32[1], xfB_p_y4.m128_f32[1]);
			printf("xfA.p.x = %f xfA.p.y = %f xfB.p.x = %f xfB.p.y = %f\n", xfA_p_x4.m128_f32[2], xfA_p_y4.m128_f32[2], xfB_p_x4.m128_f32[2], xfB_p_y4.m128_f32[2]);
			printf("xfA.p.x = %f xfA.p.y = %f xfB.p.x = %f xfB.p.y = %f\n", xfA_p_x4.m128_f32[3], xfA_p_y4.m128_f32[3], xfB_p_x4.m128_f32[3], xfB_p_y4.m128_f32[3]);
#endif

			// b2PositionSolverManifold psm;
			// psm.Initialize(pc, xfA, xfB, j);
			//b2Vec2 normal = psm.normal;

			//b2Vec2 point = psm.point;
			//float32 separation = psm.separation;
			__m128 normal_x4, normal_y4;
			__m128 point_x4, point_y4;
			__m128 separation4;
			if (type == b2Manifold::e_faceA) {
				// normal = b2Mul(xfA.q, pc->localNormal);
				normal_x4 = _mm_sub_ps(_mm_mul_ps(xfA_q_c4, localNormal_x4),
									   _mm_mul_ps(xfA_q_s4, localNormal_y4));
				normal_y4 = _mm_add_ps(_mm_mul_ps(xfA_q_s4, localNormal_x4),
									   _mm_mul_ps(xfA_q_c4, localNormal_y4));

#if B2_DEBUG_SIMD == 1
				printf("normal.x = %f normal.y = %f\n", normal_x4.m128_f32[0], normal_y4.m128_f32[0]);
				printf("normal.x = %f normal.y = %f\n", normal_x4.m128_f32[1], normal_y4.m128_f32[1]);
				printf("normal.x = %f normal.y = %f\n", normal_x4.m128_f32[2], normal_y4.m128_f32[2]);
				printf("normal.x = %f normal.y = %f\n", normal_x4.m128_f32[3], normal_y4.m128_f32[3]);
#endif

				// b2Vec2 planePoint = b2Mul(xfA, pc->localPoint);
				__m128 planePoint_x4, planePoint_y4;
				planePoint_x4 = _mm_add_ps(_mm_sub_ps(_mm_mul_ps(xfA_q_c4, localPoint_x4),
													  _mm_mul_ps(xfA_q_s4, localPoint_y4)),
										   xfA_p_x4);
				planePoint_y4 = _mm_add_ps(_mm_add_ps(_mm_mul_ps(xfA_q_s4, localPoint_x4),
													  _mm_mul_ps(xfA_q_c4, localPoint_y4)),
										   xfA_p_y4);

				// b2Vec2 clipPoint = b2Mul(xfB, pc->localPoints[index]);
				__m128 clipPoint_x4, clipPoint_y4;
				__m128 localPoints_x4 = _mm_loadu_ps(localPoints_x_array[j] + i);
				__m128 localPoints_y4 = _mm_loadu_ps(localPoints_y_array[j] + i);
				
				clipPoint_x4 = _mm_add_ps(_mm_sub_ps(_mm_mul_ps(xfB_q_c4, localPoints_x4),
													 _mm_mul_ps(xfB_q_s4, localPoints_y4)),
										  xfB_p_x4);
				clipPoint_y4 = _mm_add_ps(_mm_add_ps(_mm_mul_ps(xfB_q_s4, localPoints_x4),
													 _mm_mul_ps(xfB_q_c4, localPoints_y4)),
										  xfB_p_y4);

				// separation = b2Dot(clipPoint - planePoint, normal) - pc->radiusA - pc->radiusB;
				__m128 temp_x4 = _mm_sub_ps(clipPoint_x4, planePoint_x4);
				__m128 temp_y4 = _mm_sub_ps(clipPoint_y4, planePoint_y4);
				separation4 = _mm_sub_ps(_mm_sub_ps(_mm_add_ps(_mm_mul_ps(temp_x4, normal_x4),
															   _mm_mul_ps(temp_y4, normal_y4)),
												    radiusA4),
										 radiusB4);
				// point = clipPoint;
				point_x4 = clipPoint_x4;
				point_y4 = clipPoint_y4;
			} else if (type == b2Manifold::e_faceB) {
				//normal = b2Mul(xfB.q, pc->localNormal);
				normal_x4 = _mm_sub_ps(_mm_mul_ps(xfB_q_c4, localNormal_x4),
									   _mm_mul_ps(xfB_q_s4, localNormal_y4));
				normal_y4 = _mm_add_ps(_mm_mul_ps(xfB_q_s4, localNormal_x4),
									   _mm_mul_ps(xfB_q_c4, localNormal_y4));

				//b2Vec2 planePoint = b2Mul(xfB, pc->localPoint);
				__m128 planePoint_x4, planePoint_y4;
				planePoint_x4 = _mm_add_ps(_mm_sub_ps(_mm_mul_ps(xfB_q_c4, localPoint_x4),
													  _mm_mul_ps(xfB_q_s4, localPoint_y4)),
										   xfB_p_x4);
				planePoint_y4 = _mm_add_ps(_mm_add_ps(_mm_mul_ps(xfB_q_s4, localPoint_x4),
													  _mm_mul_ps(xfB_q_c4, localPoint_y4)),
										   xfB_p_y4);
				//b2Vec2 clipPoint = b2Mul(xfA, pc->localPoints[index]);
				__m128 clipPoint_x4, clipPoint_y4;
				__m128 localPoints_x4 = _mm_loadu_ps(localPoints_x_array[j] + i);
				__m128 localPoints_y4 = _mm_loadu_ps(localPoints_y_array[j] + i);
				
				clipPoint_x4 = _mm_add_ps(_mm_sub_ps(_mm_mul_ps(xfA_q_c4, localPoints_x4),
													 _mm_mul_ps(xfA_q_s4, localPoints_y4)),
										  xfB_p_x4);
				clipPoint_y4 = _mm_add_ps(_mm_add_ps(_mm_mul_ps(xfA_q_s4, localPoints_x4),
													 _mm_mul_ps(xfA_q_c4, localPoints_y4)),
										  xfB_p_y4);
				//separation = b2Dot(clipPoint - planePoint, normal) - pc->radiusA - pc->radiusB;
				__m128 temp_x4 = _mm_sub_ps(clipPoint_x4, planePoint_x4);
				__m128 temp_y4 = _mm_sub_ps(clipPoint_y4, planePoint_y4);
				separation4 = _mm_sub_ps(_mm_sub_ps(_mm_add_ps(_mm_mul_ps(temp_x4, normal_x4),
															   _mm_mul_ps(temp_y4, normal_y4)),
												    radiusA4),
										 radiusB4);
				//point = clipPoint;
				point_x4 = clipPoint_x4;
				point_y4 = clipPoint_y4;

				//normal = -normal;
				normal_x4 = _mm_xor_ps(normal_x4, SIGNMASK);
				normal_y4 = _mm_xor_ps(normal_y4, SIGNMASK);
			} else {
				b2Assert(false);
			}

			// b2Vec2 rA = point - cA;
			__m128 rA_x4 = _mm_sub_ps(point_x4, cA_x4);
			__m128 rA_y4 = _mm_sub_ps(point_y4, cA_y4);

			// b2Vec2 rB = point - cB;
			__m128 rB_x4 = _mm_sub_ps(point_x4, cB_x4);
			__m128 rB_y4 = _mm_sub_ps(point_y4, cB_y4);

			// minSeparation = b2Min(minSeparation, separation);
			minSeparation4 = _mm_min_ps(minSeparation4, separation4);

			// float32 C = b2Clamp(b2_baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0f);
			__m128 lowx4 = _mm_set_ps1(-b2_maxLinearCorrection);
			__m128 zero4 = _mm_set_ps1(0.0f);
			__m128 b2_baumgarte4 = _mm_set_ps1(b2_baumgarte);
			__m128 b2_linearSlop4 = _mm_set_ps1(b2_linearSlop);
			__m128 C4 = _mm_max_ps(lowx4, _mm_min_ps(_mm_mul_ps(b2_baumgarte4, _mm_add_ps(separation4, b2_linearSlop4)), zero4));

			// float32 rnA = b2Cross(rA, normal);
			__m128 rnA4 = _mm_sub_ps(_mm_mul_ps(rA_x4, normal_y4),
				                     _mm_mul_ps(rA_y4, normal_x4));
			// float32 rnB = b2Cross(rB, normal);
			__m128 rnB4 = _mm_sub_ps(_mm_mul_ps(rB_x4, normal_y4),
				                     _mm_mul_ps(rB_y4, normal_x4));
			// float32 K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
			__m128 K4 = _mm_add_ps(_mm_add_ps(_mm_add_ps(mA4, mB4),
				                              _mm_mul_ps(_mm_mul_ps(rnA4, rnA4), iA4)),
								   _mm_mul_ps(_mm_mul_ps(rnB4, rnB4), iB4));

			// float32 impulse = K > 0.0f ? - C / K : 0.0f;
			__m128 trueValue = _mm_div_ps(C4, K4);
			// negate it.
			trueValue = _mm_xor_ps(trueValue, SIGNMASK);

			 __m128 mask = _mm_cmpgt_ps(K4, zero4);
			__m128 impulse4 = _mm_or_ps(_mm_and_ps(mask, trueValue),
										_mm_andnot_ps(mask, zero4));

			// b2Vec2 P = impulse * normal;
			__m128 P_x4 = _mm_mul_ps(normal_x4, impulse4);
			__m128 P_y4 = _mm_mul_ps(normal_y4, impulse4);

#if B2_DEBUG_SIMD == 1
			printf("P.x = %f P.y = %f\n", P_x4.m128_f32[0], P_y4.m128_f32[0]);
			printf("P.x = %f P.y = %f\n", P_x4.m128_f32[1], P_y4.m128_f32[1]);
			printf("P.x = %f P.y = %f\n", P_x4.m128_f32[2], P_y4.m128_f32[2]);
			printf("P.x = %f P.y = %f\n", P_x4.m128_f32[3], P_y4.m128_f32[3]);
#endif

			// cA -= mA * P;
			cA_x4 = _mm_sub_ps(cA_x4, _mm_mul_ps(mA4, P_x4));
			cA_y4 = _mm_sub_ps(cA_y4, _mm_mul_ps(mA4, P_y4));

			// aA -= iA * b2Cross(rA, P);
			aA4 = _mm_sub_ps(aA4, _mm_mul_ps(iA4, _mm_sub_ps(_mm_mul_ps(rA_x4, P_y4), _mm_mul_ps(rA_y4, P_x4))));

			// cB += mB * P;
			cB_x4 = _mm_add_ps(cB_x4, _mm_mul_ps(mB4, P_x4));
			cB_y4 = _mm_add_ps(cB_y4, _mm_mul_ps(mB4, P_y4));

			// aB += iB * b2Cross(rB, P);
			aB4 = _mm_add_ps(aB4, _mm_mul_ps(iB4, _mm_sub_ps(_mm_mul_ps(rB_x4, P_y4), _mm_mul_ps(rB_y4, P_x4))));
		}

		// SIMD store
		_mm_storeu_ps(cA_x_array + i, cA_x4);
		_mm_storeu_ps(cA_y_array + i, cA_y4);
		_mm_storeu_ps(aA_array + i, aA4);
		_mm_storeu_ps(cB_x_array + i, cB_x4);
		_mm_storeu_ps(cB_y_array + i, cB_y4);
		_mm_storeu_ps(aB_array + i, aB4);

#if B2_DEBUG_SIMD == 1
		printf("output - cA.x = %f cA.y = %f aA = %f cB.x = %f cB.y = %f aB = %f\n",  cA_x4.m128_f32[0], cA_y4.m128_f32[0], aA4.m128_f32[0], cB_x4.m128_f32[0], cB_y4.m128_f32[0], aB4.m128_f32[0]);
		printf("output - cA.x = %f cA.y = %f aA = %f cB.x = %f cB.y = %f aB = %f\n",  cA_x4.m128_f32[1], cA_y4.m128_f32[1], aA4.m128_f32[1], cB_x4.m128_f32[1], cB_y4.m128_f32[1], aB4.m128_f32[1]);
		printf("output - cA.x = %f cA.y = %f aA = %f cB.x = %f cB.y = %f aB = %f\n",  cA_x4.m128_f32[2], cA_y4.m128_f32[2], aA4.m128_f32[2], cB_x4.m128_f32[2], cB_y4.m128_f32[2], aB4.m128_f32[2]);
		printf("output - cA.x = %f cA.y = %f aA = %f cB.x = %f cB.y = %f aB = %f\n",  cA_x4.m128_f32[3], cA_y4.m128_f32[3], aA4.m128_f32[3], cB_x4.m128_f32[3], cB_y4.m128_f32[3], aB4.m128_f32[3]);
#endif
	}

	for (int32 i = 0; i < count; ++i)
	{
		b2ContactPositionConstraint* pc = pc_array[i];
		int32 indexA = pc->indexA;
		int32 indexB = pc->indexB;

		m_positions[indexA].c.x = cA_x_array[i];
		m_positions[indexA].c.y = cA_y_array[i];
		m_positions[indexA].a = aA_array[i];

		m_positions[indexB].c.x = cB_x_array[i];
		m_positions[indexB].c.y = cB_y_array[i];
		m_positions[indexB].a = aB_array[i];
	}

	m_allocator->Free(memory);
	m_allocator->Free(localPoints_y_array);
	m_allocator->Free(localPoints_x_array);

	float32 minSeparation = 0.0f;
	minSeparation = b2Min(minSeparation, minSeparation4.m128_f32[0]);
	minSeparation = b2Min(minSeparation,  minSeparation4.m128_f32[1]);
	minSeparation = b2Min(minSeparation,  minSeparation4.m128_f32[2]);
	minSeparation = b2Min(minSeparation,  minSeparation4.m128_f32[3]);
	return minSeparation;
}

// Sequential solver.
bool b2ContactSolver::SolvePositionConstraints()
{
	float32 minSeparation = 0.0f;
	b2ContactPositionConstraint** faceA_simd_array = 
		(b2ContactPositionConstraint**)m_allocator->Allocate(m_count * sizeof(b2ContactPositionConstraint*));
	b2ContactPositionConstraint** faceB_simd_array = 
		(b2ContactPositionConstraint**)m_allocator->Allocate(m_count * sizeof(b2ContactPositionConstraint*));
	b2ContactPositionConstraint** scalar_array = 
		(b2ContactPositionConstraint**)m_allocator->Allocate(m_count * sizeof(b2ContactPositionConstraint*));
	int32 faceA_simd_index = 0;
	int32 faceB_simd_index = 0;
	int32 scalar_index = 0;
	for (int32 i = 0; i < m_count; ++i)
	{
		b2ContactPositionConstraint* pc = m_positionConstraints + i;
		// TODO(ningxin): fix the hard code pointCount.
		// SolvePositionConstraintsSIMD can handle any number of pointCount,
		// it just needs the element in array has same pointCount.
		if (pc->type == b2Manifold::e_faceA && pc->pointCount == 2)
		{
			faceA_simd_array[faceA_simd_index++] = pc;
		} if (pc->type == b2Manifold::e_faceB && pc->pointCount == 2)
		{
			faceB_simd_array[faceB_simd_index++] = pc;
		} else
		{
			scalar_array[scalar_index++] = pc;
		}
	}

	// Need to resolve the dependencies of SIMD arrays.
	

	//printf("simd_index %d scalar_index %d\n", simd_index, scalar_index);

	if (faceA_simd_index > 0)
		b2Min(minSeparation, SolvePositionConstraintsSIMD(faceA_simd_array, faceA_simd_index));
	if (faceB_simd_index > 0)
		b2Min(minSeparation, SolvePositionConstraintsSIMD(faceB_simd_array, faceB_simd_index));
	if (scalar_index > 0)
		b2Min(minSeparation, SolvePositionConstraintsScalar(scalar_array, scalar_index));
	m_allocator->Free(scalar_array);
	m_allocator->Free(faceB_simd_array);
	m_allocator->Free(faceA_simd_array);
	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -3.0f * b2_linearSlop;
}

// Sequential position solver for position constraints.
bool b2ContactSolver::SolveTOIPositionConstraints(int32 toiIndexA, int32 toiIndexB)
{
	float32 minSeparation = 0.0f;

	for (int32 i = 0; i < m_count; ++i)
	{
		b2ContactPositionConstraint* pc = m_positionConstraints + i;

		int32 indexA = pc->indexA;
		int32 indexB = pc->indexB;
		b2Vec2 localCenterA = pc->localCenterA;
		b2Vec2 localCenterB = pc->localCenterB;
		int32 pointCount = pc->pointCount;

		float32 mA = 0.0f;
		float32 iA = 0.0f;
		if (indexA == toiIndexA || indexA == toiIndexB)
		{
			mA = pc->invMassA;
			iA = pc->invIA;
		}

		float32 mB = pc->invMassB;
		float32 iB = pc->invIB;
		if (indexB == toiIndexA || indexB == toiIndexB)
		{
			mB = pc->invMassB;
			iB = pc->invIB;
		}

		b2Vec2 cA = m_positions[indexA].c;
		float32 aA = m_positions[indexA].a;

		b2Vec2 cB = m_positions[indexB].c;
		float32 aB = m_positions[indexB].a;

		// Solve normal constraints
		for (int32 j = 0; j < pointCount; ++j)
		{
			b2Transform xfA, xfB;
			xfA.q.Set(aA);
			xfB.q.Set(aB);
			xfA.p = cA - b2Mul(xfA.q, localCenterA);
			xfB.p = cB - b2Mul(xfB.q, localCenterB);

			b2PositionSolverManifold psm;
			psm.Initialize(pc, xfA, xfB, j);
			b2Vec2 normal = psm.normal;

			b2Vec2 point = psm.point;
			float32 separation = psm.separation;

			b2Vec2 rA = point - cA;
			b2Vec2 rB = point - cB;

			// Track max constraint error.
			minSeparation = b2Min(minSeparation, separation);

			// Prevent large corrections and allow slop.
			float32 C = b2Clamp(b2_toiBaugarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0f);

			// Compute the effective mass.
			float32 rnA = b2Cross(rA, normal);
			float32 rnB = b2Cross(rB, normal);
			float32 K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			// Compute normal impulse
			float32 impulse = K > 0.0f ? - C / K : 0.0f;

			b2Vec2 P = impulse * normal;

			cA -= mA * P;
			aA -= iA * b2Cross(rA, P);

			cB += mB * P;
			aB += iB * b2Cross(rB, P);
		}

		m_positions[indexA].c = cA;
		m_positions[indexA].a = aA;

		m_positions[indexB].c = cB;
		m_positions[indexB].a = aB;
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -1.5f * b2_linearSlop;
}
