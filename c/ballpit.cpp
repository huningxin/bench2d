#include <cstdio>
#include <time.h>
#include <math.h>
#include <stdlib.h>

#include "Box2D/Box2D.h"
#include "Bench2d.h"

using namespace std;

//const int e_count = 40;
const int e_count = 40;

// Simple nearest-rank %ile (on sorted array). We should have enough samples to make this reasonable.
float percentile(clock_t times[FRAMES], float pc) {
  int rank = (int)((pc * FRAMES) / 100);
  return times[rank];
}

int _cmp(const void *a, const void *b) {
	return *(clock_t*)a - *(clock_t*)b;
}

result_t measure(clock_t times[FRAMES]) {
  float values[FRAMES];
  result_t r;

	float total = 0;
    int32 frameCount = b2Params::frame1 ? 1 : (b2Params::frame10 ? 10 : FRAMES);
	for (int i = 0; i < frameCount; ++i) {
		values[i] = (float)times[i] / CLOCKS_PER_SEC * 1000;
	 	total += values[i];
	}
  r.mean = total / frameCount;

	qsort(times, frameCount, sizeof(clock_t), _cmp);
  r.pc_5th = percentile(times, 5) / CLOCKS_PER_SEC * 1000;
  r.pc_95th = percentile(times, 95) / CLOCKS_PER_SEC * 1000;
  return r;
}

result_t ballpit(int argc, char **argv) {
    b2Params::init(argc, argv);    
	// Define the gravity vector.
	b2Vec2 gravity(0.0f, -10.0f);

	// Construct a world object, which will hold and simulate the rigid bodies.
	b2World world(gravity);
    world.SetAllowSleeping(false);

    b2Body* ground = NULL;
    {
        b2BodyDef bd;
        ground = world.CreateBody(&bd);
    }

    {
        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.allowSleep = false;
        bd.position.Set(0.0f, 10.0f);
        b2Body* body = world.CreateBody(&bd);

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
        (b2RevoluteJoint*)world.CreateJoint(&jd);
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
		  b2Body *body = world.CreateBody(&bd);
		  body->CreateFixture(&shape, 5.0f);
        }
      }
    }

    if (!b2Params::frame1 && !b2Params::frame10) {
	  for (int32 i = 0; i < WARMUP; ++i) {
		world.Step(1.0f/60.0f, 3, 3);
      }
    }

	clock_t times[FRAMES];

    int32 frameCount = b2Params::frame1 ? 1 : (b2Params::frame10 ? 10 : FRAMES);
	for (int32 i = 0; i < frameCount; ++i) {
		clock_t start = clock();
		world.Step(1.0f/60.0f, 3, 3);
		clock_t end = clock();
		times[i] = end - start;
        if (b2Params::debug) {
		    printf("%f\n", (float32)(end - start) / CLOCKS_PER_SEC * 1000);
        }
	}

  b2Counters::dump();
  b2Cycles::dump();
  return measure(times);
}
