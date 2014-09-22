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

#include <Box2D/Common/b2Settings.h>
#include <cstdlib>
#include <cstdio>
#include <cstdarg>
#include <string.h>

b2Version b2_version = {2, 2, 1};

// Memory allocators. Modify these to use your own allocator.
void* b2Alloc(int32 size)
{
	return malloc(size);
}

void b2Free(void* mem)
{
	free(mem);
}

// You can modify this to use your logging facility.
void b2Log(const char* string, ...)
{
	va_list args;
	va_start(args, string);
	vprintf(string, args);
	va_end(args);
}

bool b2Params::useSimd = false;
void b2Params::init(int argc, char **argv) {
  if (argc <= 1) {
    return;
  }
  if (strcmp(argv[1], "simd") == 0) {
    useSimd = true;
  }
  if (strcmp(argv[1], "nosimd") == 0) {
    useSimd = false;
  }
}

int b2Counters::solvePositionConstraints;
int b2Counters::pointCountsEqual;
int b2Counters::pointCountsNotEqual;
int b2Counters::pointCount1;
int b2Counters::pointCount2;
int b2Counters::pointCountOther;

void b2Counters::dump() {
    b2Log("solvePositionConstraints: %d\n", solvePositionConstraints);
    b2Log("pointCountsEqual:         %d\n", pointCountsEqual);
    b2Log("pointCountsNotEqual:      %d\n", pointCountsNotEqual);
    b2Log("pointCount1:              %d\n", pointCount1);
    b2Log("pointCount2:              %d\n", pointCount2);
    b2Log("pointCountOther:          %d\n", pointCountOther);
}
