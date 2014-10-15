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

#ifndef DEMO01_H
#define DEMO01_H

class Demo01 : public Test
{
public:

    Demo01()
    {
      b2BodyDef bd;

      // Create a stick body
      bd.type = b2_staticBody;
      bd.position.Set(0.0f, 2.0f);
      b2Body *stick = m_world->CreateBody(&bd);

      // Attach a rectangle fixture to the stick
      b2PolygonShape rect;
      rect.SetAsBox(0.5f, 10.0f, b2Vec2(0.0f, 0.0f), 70.0f * b2_pi/180.0f);
      stick->CreateFixture(&rect, 0.0f);

      // Create a ball body
      bd.type = b2_dynamicBody;
      bd.position.Set(0.0f, 20.0f);
      b2Body *ball = m_world->CreateBody(&bd);

      // Attach a circle fixture to the ball
      b2CircleShape circle;
      circle.m_radius = 2.0f;
      ball->CreateFixture(&circle, 5.0f);
    }

    void Step(Settings* settings)
    {
        Test::Step(settings);
    }

    static Test* Create()
    {
        return new Demo01;
    }

    b2RevoluteJoint* m_joint;
};

#endif
