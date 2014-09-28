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

result_t bench(int argc, char **argv) {
    b2Params::init(argc, argv);    
	// Define the gravity vector.
	b2Vec2 gravity(0.0f, -10.0f);

	// Construct a world object, which will hold and simulate the rigid bodies.
	b2World world(gravity);
  world.SetAllowSleeping(false);

	{
		b2BodyDef bd;
		b2Body* ground = world.CreateBody(&bd);

		b2EdgeShape shape;
		shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
		ground->CreateFixture(&shape, 0.0f);
	}

  b2Body* topBody = NULL;

	{
		float32 a = 0.5f;
		b2PolygonShape shape;
		shape.SetAsBox(a, a);

		b2Vec2 x(-7.0f, 0.75f);
		b2Vec2 y;
		b2Vec2 deltaX(0.5625f, 1);
		b2Vec2 deltaY(1.125f, 0.0f);

		for (int32 i = 0; i < e_count; ++i) {
			y = x;

			for (int32 j = i; j < e_count; ++j) {
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position = y;
				b2Body *body = world.CreateBody(&bd);
				body->CreateFixture(&shape, 5.0f);

        topBody = body;

				y += deltaY;
			}

			x += deltaX;
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
            printf("%f :: ", topBody->GetPosition().y);
		    printf("%f\n", (float32)(end - start) / CLOCKS_PER_SEC * 1000);
        }
	}

  b2Counters::dump();
  b2Cycles::dump();
  return measure(times);
}
