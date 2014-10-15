/*
* Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#ifndef BALLPIT_H
#define BALLPIT_H

class BallPit : public Test
{
public:

    enum
    {
        e_count = 40
    };

    BallPit()
    {
        b2Body* ground = NULL;
        {
            b2BodyDef bd;
            ground = m_world->CreateBody(&bd);
        }

        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.allowSleep = false;
            bd.position.Set(0.0f, 10.0f);
            b2Body* body = m_world->CreateBody(&bd);

            b2PolygonShape shape;
            shape.SetAsBox(0.5f, 10.0f, b2Vec2( 10.0f, 0.0f), 0.0);
            body->CreateFixture(&shape, 5.0f);
            shape.SetAsBox(0.5f, 10.0f, b2Vec2(-10.0f, 0.0f), 0.0);
            body->CreateFixture(&shape, 5.0f);
            shape.SetAsBox(10.0f, 0.5f, b2Vec2(0.0f, 10.0f), 0.0);
            body->CreateFixture(&shape, 5.0f);
            shape.SetAsBox(10.0f, 0.5f, b2Vec2(0.0f, -10.0f), 0.0);
            body->CreateFixture(&shape, 5.0f);

            b2RevoluteJointDef jd;
            jd.bodyA = ground;
            jd.bodyB = body;
            jd.localAnchorA.Set(0.0f, 10.0f);
            jd.localAnchorB.Set(0.0f, 0.0f);
            jd.referenceAngle = 0.0f;
            jd.motorSpeed = 0.10f * b2_pi;
            jd.maxMotorTorque = 1e8f;
            jd.enableMotor = true;
            m_joint = (b2RevoluteJoint*)m_world->CreateJoint(&jd);
        }

        // add the balls in a pyramid
        {
          float32 step = (20.0f - 1.0f)/e_count;
          float32 radius = step/2 - 0.10f;
          b2CircleShape shape;
          shape.m_radius = radius;

          b2Vec2  bottomLeft(-9.5f + radius, 0.5f + radius);
          for (int32 y = 0; y < e_count; ++y) {
            for (int32 x = y; x < e_count; ++x) {
              b2Vec2 offset(x*step - y*step/2, y*step);
              b2BodyDef bd;
     		  bd.type = b2_dynamicBody;
			  bd.position = bottomLeft + offset;
			  b2Body *body = m_world->CreateBody(&bd);
			  body->CreateFixture(&shape, 5.0f);
            }
          }
        }
    }

    void Step(Settings* settings)
    {
        Test::Step(settings);
    }

    static Test* Create()
    {
        return new BallPit;
    }

    b2RevoluteJoint* m_joint;
};

#endif
