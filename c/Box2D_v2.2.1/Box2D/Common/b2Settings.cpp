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
#include <intrin.h>

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

bool b2Params::simd    = false;
bool b2Params::dumpPos = false;
bool b2Params::dumpCon = false;
bool b2Params::testOut = false;
bool b2Params::debug   = false;
bool b2Params::frame1  = false;
bool b2Params::frame10 = false;
bool b2Params::sortCon = false;

void b2Params::init(int argc, char **argv) {
  if (argc <= 1) {
    return;
  }
  for (int32 i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "simd") == 0) {
      simd = true;
    }
    if (strcmp(argv[i], "dump") == 0) {
      dumpPos = true;
    }
    if (strcmp(argv[i], "dumpCon") == 0) {
      dumpCon = true;
    }
    if (strcmp(argv[i], "testOut") == 0) {
      testOut = true;
    }
    if (strcmp(argv[i], "debug") == 0) {
      debug = true;
    }
    if (strcmp(argv[i], "frame1") == 0) {
      frame1 = true;
      frame10 = false;
    }
    if (strcmp(argv[i], "frame10") == 0) {
      frame10 = true;
      frame1  = false;
    }
    if (strcmp(argv[i], "sortCon") == 0) {
      sortCon = true;
    }
  }
}

int b2Counters::solvePositionConstraints = 0;
int b2Counters::pointCountsEqual         = 0;
int b2Counters::pointCountsNotEqual      = 0;
int b2Counters::pointCount1              = 0;
int b2Counters::pointCount2              = 0;
int b2Counters::pointCountOther          = 0;
int b2Counters::indexOverlap             = 0;
int b2Counters::noIndexOverlap           = 0;
int b2Counters::minSeparationOk          = 0;

void b2Counters::dump() {
    b2Log("solvePositionConstraints: %d\n", solvePositionConstraints);
    b2Log("pointCountsEqual:         %d\n", pointCountsEqual);
    b2Log("pointCountsNotEqual:      %d\n", pointCountsNotEqual);
    b2Log("pointCount1:              %d\n", pointCount1);
    b2Log("pointCount2:              %d\n", pointCount2);
    b2Log("indexOverlap:             %d\n", indexOverlap);
    b2Log("noIndexOverlap:           %d\n", noIndexOverlap);
    b2Log("minSeparationOk:          %d\n", minSeparationOk);
}

b2Cycles::b2Cycles(int32 cycleIndex, char *cycleName) {
  m_currentIndex = cycleIndex;
  m_cycles[cycleIndex].cycleName = cycleName;
  m_cycles[cycleIndex].start     = __rdtsc();
}

b2Cycles::~b2Cycles() {
  unsigned __int64 stop = __rdtsc();
  m_cycles[m_currentIndex].total += (stop - m_cycles[m_currentIndex].start);
}

void b2Cycles::dump() {
  printf("Cycles:\n");
  for (int32 i = 0; i < m_maxCycles; ++i) {
    if (m_cycles[i].cycleName != NULL) {
      double milCycles = ((double)m_cycles[i].total)/1000000.0;
      printf("%s: %6.2fM\n", m_cycles[i].cycleName, milCycles);
    }
  }
}

b2Cycles::cycleData b2Cycles::m_cycles[b2Cycles::m_maxCycles];
