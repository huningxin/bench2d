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

#include <Box2D/Dynamics/Contacts/b2ContactSolver.h>

#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Common/b2StackAllocator.h>

#define B2_DEBUG_SOLVER 0

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
    if (b2Params::sortCon) {
      SortPositionConstraints();
    }
    TEST_COND(b2Params::dumpCon, {
      TEST_PRINTF(("PositionConstraints:\n"));
      DumpPositionConstraints();
    });
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
        TEST_PRINTF(("psm.separation: %10.4f\n", separation));
    }

	b2Vec2 normal;
	b2Vec2 point;
	float32 separation;
};

bool b2ContactSolver::IndexOverlap(int32 indexA[4], int32 indexB[4]) {
  int32 values[8] = {indexA[0], indexA[1], indexA[2], indexA[3],
                     indexB[0], indexB[1], indexB[2], indexB[3]};
  for (int32 i = 0; i < 7; ++i) {
    for (int32 j = i+1; j < 8; ++j) {
      if (values[i] == values[j]) {
        return true;
      }
    }
  }
  return false;
}

// Sequential solver.
float32 b2ContactSolver::SimdSolvePositionConstraints()
{
	float32 minSeparation = 0.0f;
    __m128 b2_baumgarte4               = _mm_set_ps(b2_baumgarte, b2_baumgarte, b2_baumgarte, b2_baumgarte);
    __m128 b2_linearSlop4              = _mm_set_ps(b2_linearSlop, b2_linearSlop, b2_linearSlop, b2_linearSlop);
    __m128 neg_b2_maxLinearCorrection4 = _mm_set_ps(-b2_maxLinearCorrection, -b2_maxLinearCorrection, -b2_maxLinearCorrection, -b2_maxLinearCorrection);
    __m128 zero4                       = _mm_set_ps(0.0f, 0.0f, 0.0f, 0.0f);
    __m128 signMask4                   = _mm_castsi128_ps(_mm_set_epi32(0x80000000, 0x80000000, 0x80000000, 0x80000000));

	for (int32 i = 0; i < (m_count-3); i+=4)
	{
		b2ContactPositionConstraint *pc = m_positionConstraints + i;

        int32 indexA[4] = {pc->indexA, (pc+1)->indexA, (pc+2)->indexA, (pc+3)->indexA};
        int32 indexB[4] = {pc->indexB, (pc+1)->indexB, (pc+2)->indexB, (pc+3)->indexB};
        if (IndexOverlap(indexA, indexB)) {
          // doesn't deal with aliasing between the 4 lanes
          COUNTER_INC(indexOverlap);
          float32 minSep = SolveHelper(i, 4);
          minSeparation = b2Min(minSeparation, minSep);
          continue;
        }
        else {
          COUNTER_INC(noIndexOverlap);
        }
        b2Vec2 localCenterA[4] = {pc->localCenterA, (pc+1)->localCenterA,
                                  (pc+2)->localCenterA, (pc+3)->localCenterA};
        b2Vec2 localCenterB[4] = {pc->localCenterB, (pc+1)->localCenterB,
                                 (pc+2)->localCenterB, (pc+3)->localCenterB};
        int32 pointCount[4] = {pc->pointCount, (pc+1)->pointCount, (pc+2)->pointCount, (pc+3)->pointCount};

        b2Vec2  cA[4];
		float32 aA[4];
		b2Vec2  cB[4];
		float32 aB[4];

        float32 mA[4] = {pc->invMassA, (pc+1)->invMassA, (pc+2)->invMassA, (pc+3)->invMassA};
        float32 iA[4] = {pc->invIA, (pc+1)->invIA, (pc+2)->invIA, (pc+3)->invIA};
        float32 mB[4] = {pc->invMassB, (pc+1)->invMassB, (pc+2)->invMassB, (pc+3)->invMassB};
        float32 iB[4] = {pc->invIB, (pc+1)->invIB, (pc+2)->invIB, (pc+3)->invIB};

		// Solve normal constraints
        // if all 4 have the same point count we're good, otherwise they'll have to be done
        // one-by-one
        if ((pointCount[0] == pointCount[1]) &&
            (pointCount[0] == pointCount[2]) &&
            (pointCount[0] == pointCount[3])) {
          COUNTER_INC(pointCountsEqual);
          COUNTER_COND_INC(pointCount[0] == 1, pointCount1);
          COUNTER_COND_INC(pointCount[0] == 2, pointCount2);
          COUNTER_COND_INC(pointCount[0] != 1 && pointCount[0] != 2, pointCountOther);
          if (pointCount[0] == 1) {
            cA[0] = m_positions[indexA[0]].c;
            cA[1] = m_positions[indexA[1]].c;
            cA[2] = m_positions[indexA[2]].c;
            cA[3] = m_positions[indexA[3]].c;
		    cB[0] = m_positions[indexB[0]].c;
            cB[1] = m_positions[indexB[1]].c;
            cB[2] = m_positions[indexB[2]].c;
            cB[3] = m_positions[indexB[3]].c;
            __m128 aA4 = _mm_set_ps(m_positions[indexA[3]].a, m_positions[indexA[2]].a, m_positions[indexA[1]].a, m_positions[indexA[0]].a);
            __m128 aB4 = _mm_set_ps(m_positions[indexB[3]].a, m_positions[indexB[2]].a, m_positions[indexB[1]].a, m_positions[indexB[0]].a);

            __m128 cAx4 = _mm_set_ps(cA[3].x, cA[2].x, cA[1].x, cA[0].x);
            __m128 cAy4 = _mm_set_ps(cA[3].y, cA[2].y, cA[1].y, cA[0].y);
            __m128 cBx4 = _mm_set_ps(cB[3].x, cB[2].x, cB[1].x, cB[0].x);
            __m128 cBy4 = _mm_set_ps(cB[3].y, cB[2].y, cB[1].y, cB[0].y);

            __m128 mA4 = _mm_loadu_ps(mA);
            __m128 mB4 = _mm_loadu_ps(mB);
            __m128 iA4 = _mm_loadu_ps(iA);
            __m128 iB4 = _mm_loadu_ps(iB);

                b2Transform xfA[4], xfB[4];
			    xfA[0].q.Set(aA4.m128_f32[0]);
			    xfA[1].q.Set(aA4.m128_f32[1]);
			    xfA[2].q.Set(aA4.m128_f32[2]);
			    xfA[3].q.Set(aA4.m128_f32[3]);

                xfB[0].q.Set(aB4.m128_f32[0]);
                xfB[1].q.Set(aB4.m128_f32[1]);
                xfB[2].q.Set(aB4.m128_f32[2]);
                xfB[3].q.Set(aB4.m128_f32[3]);

			    xfA[0].p = cA[0] - b2Mul(xfA[0].q, localCenterA[0]);
			    xfA[1].p = cA[1] - b2Mul(xfA[1].q, localCenterA[1]);
			    xfA[2].p = cA[2] - b2Mul(xfA[2].q, localCenterA[2]);
			    xfA[3].p = cA[3] - b2Mul(xfA[3].q, localCenterA[3]);

			    xfB[0].p = cB[0] - b2Mul(xfB[0].q, localCenterB[0]);
			    xfB[1].p = cB[1] - b2Mul(xfB[1].q, localCenterB[1]);
			    xfB[2].p = cB[2] - b2Mul(xfB[2].q, localCenterB[2]);
			    xfB[3].p = cB[3] - b2Mul(xfB[3].q, localCenterB[3]);

                b2Transform4 xfA4, xfB4;

			    b2PositionSolverManifold psm[4];
			    psm[0].Initialize(pc+0, xfA[0], xfB[0], 0);
			    psm[1].Initialize(pc+1, xfA[1], xfB[1], 0);
			    psm[2].Initialize(pc+2, xfA[2], xfB[2], 0);
			    psm[3].Initialize(pc+3, xfA[3], xfB[3], 0);

                __m128 normalx4   = _mm_set_ps(psm[3].normal.x, psm[2].normal.x, psm[1].normal.x, psm[0].normal.x);
                __m128 normaly4   = _mm_set_ps(psm[3].normal.y, psm[2].normal.y, psm[1].normal.y, psm[0].normal.y);

                __m128 pointx4 = _mm_set_ps(psm[3].point.x, psm[2].point.x, psm[1].point.x, psm[0].point.x);
                __m128 pointy4 = _mm_set_ps(psm[3].point.y, psm[2].point.y, psm[1].point.y, psm[0].point.y);

                __m128 separation4 = _mm_set_ps(psm[3].separation, psm[2].separation, psm[1].separation, psm[0].separation);

                __m128 rAx4 = _mm_sub_ps(pointx4, cAx4);
                __m128 rAy4 = _mm_sub_ps(pointy4, cAy4);
                __m128 rBx4 = _mm_sub_ps(pointx4, cBx4);
                __m128 rBy4 = _mm_sub_ps(pointy4, cBy4);

			    // Track max constraint error.
			    minSeparation = b2Min(minSeparation, separation4.m128_f32[0]);
			    minSeparation = b2Min(minSeparation, separation4.m128_f32[1]);
			    minSeparation = b2Min(minSeparation, separation4.m128_f32[2]);
			    minSeparation = b2Min(minSeparation, separation4.m128_f32[3]);

			    // Prevent large corrections and allow slop.
                __m128 C4 = b2Clamp4(_mm_mul_ps(b2_baumgarte4, _mm_add_ps(separation4, b2_linearSlop4)),
                                     neg_b2_maxLinearCorrection4, zero4);

			    // Compute the effective mass.
                __m128 rnA4 = b2Cross4(rAx4, rAy4, normalx4, normaly4);
                __m128 rnB4 = b2Cross4(rBx4, rBy4, normalx4, normaly4);
                __m128 K4 = _mm_add_ps(
                              mA4,
                              _mm_add_ps(
                                mB4,
                                _mm_add_ps(
                                  _mm_mul_ps(
                                    iA4,
                                    _mm_mul_ps(rnA4, rnA4)),
                                  _mm_mul_ps(
                                    iB4,
                                    _mm_mul_ps(rnB4, rnB4)))));

			    // Compute normal impulse
                __m128 cmpgt    = _mm_cmpgt_ps(K4, zero4);
                __m128 negdiv4  = _mm_xor_ps(signMask4, _mm_div_ps(C4, K4));
                __m128 impulse4 = _mm_or_ps(_mm_and_ps(cmpgt, negdiv4), _mm_andnot_ps(cmpgt, zero4));
                __m128 Px4 = _mm_mul_ps(impulse4, normalx4);
                __m128 Py4 = _mm_mul_ps(impulse4, normaly4);

                cAx4 = _mm_sub_ps(cAx4, _mm_mul_ps(mA4, Px4));
                cAy4 = _mm_sub_ps(cAy4, _mm_mul_ps(mA4, Py4));
                aA4  = _mm_sub_ps(aA4, _mm_mul_ps(iA4, b2Cross4(rAx4, rAy4, Px4, Py4)));

                cBx4 = _mm_add_ps(cBx4, _mm_mul_ps(mB4, Px4));
                cBy4 = _mm_add_ps(cBy4, _mm_mul_ps(mB4, Py4));
                aB4  = _mm_add_ps(aB4, _mm_mul_ps(iB4, b2Cross4(rBx4, rBy4, Px4, Py4)));

                m_positions[indexA[0]].c.x = cAx4.m128_f32[0];
                m_positions[indexA[1]].c.x = cAx4.m128_f32[1];
                m_positions[indexA[2]].c.x = cAx4.m128_f32[2];
                m_positions[indexA[3]].c.x = cAx4.m128_f32[3];
                m_positions[indexA[0]].c.y = cAy4.m128_f32[0];
                m_positions[indexA[1]].c.y = cAy4.m128_f32[1];
                m_positions[indexA[2]].c.y = cAy4.m128_f32[2];
                m_positions[indexA[3]].c.y = cAy4.m128_f32[3];

                m_positions[indexA[0]].a = aA4.m128_f32[0];
                m_positions[indexA[1]].a = aA4.m128_f32[1];
                m_positions[indexA[2]].a = aA4.m128_f32[2];
                m_positions[indexA[3]].a = aA4.m128_f32[3];

                m_positions[indexB[0]].c.x = cBx4.m128_f32[0];
                m_positions[indexB[1]].c.x = cBx4.m128_f32[1];
                m_positions[indexB[2]].c.x = cBx4.m128_f32[2];
                m_positions[indexB[3]].c.x = cBx4.m128_f32[3];
                m_positions[indexB[0]].c.y = cBy4.m128_f32[0];
                m_positions[indexB[1]].c.y = cBy4.m128_f32[1];
                m_positions[indexB[2]].c.y = cBy4.m128_f32[2];
                m_positions[indexB[3]].c.y = cBy4.m128_f32[3];

                m_positions[indexB[0]].a = aB4.m128_f32[0];
                m_positions[indexB[1]].a = aB4.m128_f32[1];
                m_positions[indexB[2]].a = aB4.m128_f32[2];
                m_positions[indexB[3]].a = aB4.m128_f32[3];
          }
          else {
            for (int32 i4 = 0; i4 < 4; ++i4) {
              cA[i4] = m_positions[indexA[i4]].c;
              aA[i4] = m_positions[indexA[i4]].a;
              cB[i4] = m_positions[indexB[i4]].c;
              aB[i4] = m_positions[indexB[i4]].a;
		      for (int32 j = 0; j < pointCount[i4]; ++j)
		      {
			      b2Transform xfA, xfB;
			      xfA.q.Set(aA[i4]);
			      xfB.q.Set(aB[i4]);
			      xfA.p = cA[i4] - b2Mul(xfA.q, localCenterA[i4]);
			      xfB.p = cB[i4] - b2Mul(xfB.q, localCenterB[i4]);

			      b2PositionSolverManifold psm;
			      psm.Initialize(pc+i4, xfA, xfB, j);
			      b2Vec2 normal = psm.normal;

			      b2Vec2 point = psm.point;
			      float32 separation = psm.separation;

			      b2Vec2 rA = point - cA[i4];
			      b2Vec2 rB = point - cB[i4];

			      // Track max constraint error.
			      minSeparation = b2Min(minSeparation, separation);

			      // Prevent large corrections and allow slop.
			      float32 C = b2Clamp(b2_baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0f);

			      // Compute the effective mass.
			      float32 rnA = b2Cross(rA, normal);
			      float32 rnB = b2Cross(rB, normal);
			      float32 K = mA[i4] + mB[i4] + iA[i4] * rnA * rnA + iB[i4] * rnB * rnB;

			      // Compute normal impulse
			      float32 impulse = K > 0.0f ? - C / K : 0.0f;

			      b2Vec2 P = impulse * normal;

			      cA[i4] -= mA[i4] * P;
			      aA[i4] -= iA[i4] * b2Cross(rA, P);

			      cB[i4] += mB[i4] * P;
			      aB[i4] += iB[i4] * b2Cross(rB, P);
		      }
              m_positions[indexA[i4]].c = cA[i4];
              m_positions[indexA[i4]].a = aA[i4];
              m_positions[indexB[i4]].c = cB[i4];
              m_positions[indexB[i4]].a = aB[i4];
            }
          }
        }
        else {
          COUNTER_INC(pointCountsNotEqual);
          for (int32 i4 = 0; i4 < 4; ++i4) {
            cA[i4] = m_positions[indexA[i4]].c;
            aA[i4] = m_positions[indexA[i4]].a;
            cB[i4] = m_positions[indexB[i4]].c;
            aB[i4] = m_positions[indexB[i4]].a;
		    for (int32 j = 0; j < pointCount[i4]; ++j)
		    {
			    b2Transform xfA, xfB;
			    xfA.q.Set(aA[i4]);
			    xfB.q.Set(aB[i4]);
			    xfA.p = cA[i4] - b2Mul(xfA.q, localCenterA[i4]);
			    xfB.p = cB[i4] - b2Mul(xfB.q, localCenterB[i4]);

			    b2PositionSolverManifold psm;
			    psm.Initialize(pc+i4, xfA, xfB, j);
			    b2Vec2 normal = psm.normal;

			    b2Vec2 point = psm.point;
			    float32 separation = psm.separation;

			    b2Vec2 rA = point - cA[i4];
			    b2Vec2 rB = point - cB[i4];

			    // Track max constraint error.
			    minSeparation = b2Min(minSeparation, separation);

			    // Prevent large corrections and allow slop.
			    float32 C = b2Clamp(b2_baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0f);

			    // Compute the effective mass.
			    float32 rnA = b2Cross(rA, normal);
			    float32 rnB = b2Cross(rB, normal);
			    float32 K = mA[i4] + mB[i4] + iA[i4] * rnA * rnA + iB[i4] * rnB * rnB;

			    // Compute normal impulse
			    float32 impulse = K > 0.0f ? - C / K : 0.0f;

			    b2Vec2 P = impulse * normal;

			    cA[i4] -= mA[i4] * P;
			    aA[i4] -= iA[i4] * b2Cross(rA, P);

			    cB[i4] += mB[i4] * P;
			    aB[i4] += iB[i4] * b2Cross(rB, P);
		    }
            m_positions[indexA[i4]].c = cA[i4];
            m_positions[indexA[i4]].a = aA[i4];
            m_positions[indexB[i4]].c = cB[i4];
            m_positions[indexB[i4]].a = aB[i4];
          }
        }

	}
    
    int32 remCount = m_count & 3;
    if (remCount > 0) {
      float32 minSep = SolveHelper(m_count & ~3, remCount);
      minSeparation = b2Min(minSep, minSeparation);
    }
	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation;
}

float32 b2ContactSolver::SolveHelper(int32 startIndex, int32 count) {
    float32 minSeparation = 0.0f;

	for (int32 i = startIndex; i < startIndex + count; ++i)
	{
		b2ContactPositionConstraint* pc = m_positionConstraints + i;

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
			float32 C = b2Clamp(b2_baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0f);

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
    return minSeparation;
}

// Sequential solver.
bool b2ContactSolver::SolvePositionConstraints()
{
    COUNTER_INC(solvePositionConstraints);
    b2Cycles cycles(0, "SolvePositionConstrains");
    float32 minSeparation;
    if (b2Params::simd) {
      b2Cycles cycles(1, "SimdSolvePositionConstraints");
      minSeparation = SimdSolvePositionConstraints();
    }
    else {
      b2Cycles cycles(2, "nonSimdSolvePositionConstraints");
      minSeparation = SolveHelper(0, m_count);
    }

    TEST_BLOCK({
      TEST_PRINTF(("minSeparation: %10.4f\n", minSeparation));
      if (b2Params::dumpPos) {
        TEST_PRINTF(("Positions:\n"));
        DumpPositions();
      }
    });

    bool minSeparationOk = minSeparation >= -3.0f * b2_linearSlop;
    if (minSeparationOk) {
      COUNTER_INC(minSeparationOk);
    }
  	return minSeparationOk;
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

bool b2ContactSolver::IsUnique(int32 first, int32 last, int32 check, int32 *sorted) {
  b2ContactPositionConstraint* pcCheck = m_positionConstraints + sorted[check];
  int32 iACheck = pcCheck->indexA;
  int32 iBCheck = pcCheck->indexB;
  for (int32 index = first; index <= last; ++index) {
    b2ContactPositionConstraint* pc = m_positionConstraints + sorted[index];
    int32 iA = pc->indexA;
    int32 iB = pc->indexB;
    if (iA == iACheck || iA == iBCheck || 
        iB == iACheck || iB == iBCheck) {
      return false;
    }
  }
  return true;
}

bool b2ContactSolver::Find4Uniques(int32 index, int32 *sorted) {
  for (int32 i = index+1; i < index+4; ++i) {
    int32 j = i;
    while ( j < m_count && !IsUnique(index, i-1, j, sorted)) {
      j++;
    }
    if (j < m_count && j != i) {
      int32 temp = sorted[i];
      sorted[i] = sorted[j];
      sorted[j] = temp;
    }
    else if (j == m_count) {
      return false;
    }
  }
  return true;
}

void b2ContactSolver::Swap(int32 index1, int32 index2) {
  b2ContactPositionConstraint *pc1 = m_positionConstraints + index1;
  b2ContactPositionConstraint *pc2 = m_positionConstraints + index2;
  b2ContactPositionConstraint temp;
  temp = *pc1;
  *pc1 = *pc2;
  *pc2 = temp;
}

void b2ContactSolver::Dump4(int32 index) {
  for (int32 i = index; i < index+4; ++i) {
    b2ContactPositionConstraint *pc = m_positionConstraints + i;
    b2Log("%4d. indexA: %4d, indexB: %4d\n", i, pc->indexA, pc->indexB);
  }
}

void b2ContactSolver::Dump4Sorted(int32 index, int32 *sorted) {
  for (int32 i = index; i < index+4; ++i) {
    b2ContactPositionConstraint *pc = m_positionConstraints + sorted[i];
    b2Log("%4d(%4d). indexA: %4d, indexB: %4d\n", sorted[i], i, pc->indexA, pc->indexB);
  }
}

void b2ContactSolver::DumpPos4(int32 index) {
  for (int32 i = index; i < index+4; ++i) {
    b2ContactPositionConstraint *pc = m_positionConstraints + i;
    int32 iA = pc->indexA;
    int32 iB = pc->indexB;
    b2Position *pA = &(m_positions[iA]);
    b2Position *pB = &(m_positions[iB]);
    b2Log("A %4d(%4d). c: (%8.2f, %8.2f) a: %8.2f\n", i, iA, pA->c.x, pA->c.y, pA->a);
    b2Log("B %4d(%4d). c: (%8.2f, %8.2f) a: %8.2f\n", i, iB, pB->c.x, pB->c.y, pB->a);
  }
}

void b2ContactSolver::SortPositionConstraints() {
  int32 *sorted = (int32 *)m_allocator->Allocate(m_count * sizeof(int32));
  int32 i;
  for (i = 0; i < m_count; ++i) {
    sorted[i] = i;
  }
  for (i = 0; i < m_count-3; i += 4) {
    bool found = Find4Uniques(i, sorted); 
    if (!found) {
      break;
    }
  }

  b2ContactPositionConstraint *copy = (b2ContactPositionConstraint*)m_allocator->Allocate(m_count * sizeof(b2ContactPositionConstraint));
  memcpy(copy, m_positionConstraints, m_count *sizeof (b2ContactPositionConstraint));
  for (i = 0; i < m_count; ++i) {
    if (sorted[i] != i) {
      memcpy(m_positionConstraints + i, copy + sorted[i], sizeof (b2ContactPositionConstraint));
    }
  }
  m_allocator->Free(copy);
  m_allocator->Free(sorted);
}

void b2ContactSolver::DumpPositions() {
  for (int32 i = 0; i < m_count; ++i) { // TODO: m_count is NOT the number of positions
    b2Position *p = &(m_positions[i]);
    b2Log("%4d. c: (%8.2f, %8.2f) a: %8.2f\n", i, p->c.x, p->c.y, p->a);
  }
}

void b2ContactSolver::DumpPositionConstraints() {
  int32 i;
  int32 indexOverlapCount   = 0;
  int32 indexNoOverlapCount  = 0;

  for (i = 0; i < m_count-3; i += 4) {
    b2ContactPositionConstraint* pc = m_positionConstraints + i;
    int32 indexA[4] = {pc->indexA, (pc+1)->indexA, (pc+2)->indexA, (pc+3)->indexA};
    int32 indexB[4] = {pc->indexB, (pc+1)->indexB, (pc+2)->indexB, (pc+3)->indexB};
    bool  overlaps = IndexOverlap(indexA, indexB);
    if (overlaps) {
      indexOverlapCount++;
    }
    else {
      indexNoOverlapCount++;
    }
    for (int32 i4 = 0; i4 < 4; ++i4) {
      pc = m_positionConstraints + (i+i4);
      b2Log("%4d. (indexA: %4d, indexB: %4d, pointCount: %4d)", i, pc->indexA, pc->indexB, pc->pointCount);
      if (overlaps) {
        b2Log(" OVERLAP");
      }
      b2Log("\n");
    }
  }
  for (i = m_count & ~3; i < m_count; ++i) {
    b2ContactPositionConstraint* pc = m_positionConstraints + i;
    b2Log("%4d. (indexA: %4d, indexB: %4d, pointCount: %4d)\n", i, pc->indexA, pc->indexB, pc->pointCount);
  }
  b2Log("indexOverlapCount:   %6d\n", indexOverlapCount);
  b2Log("indexNoOverlapCount:  %6d\n", indexNoOverlapCount);
}
