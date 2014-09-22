#include <stdio.h>
#include <emmintrin.h>
#include <xmmintrin.h>

typedef float float32;

struct b2Vec2 {
  float32 x;
  float32 y;
};

struct b2Rot {
  float32 s;
  float32 c;
};

struct b2Transform {
	b2Vec2 p;
	b2Rot  q;
};

static void print_ps(__m128 m128) {
  float32 f4[4];
  _mm_storeu_ps(f4, m128);
  printf("%.2f, %.2f, %.2f, %.2f", f4[0], f4[1], f4[2], f4[3]);
}

static void reportVal(char * msg, __m128 m128) {
  printf("%s: ", msg);
  print_ps(m128);
  printf("\n");
}

inline b2Vec2 b2MulSimd(const b2Transform& T, const b2Vec2& v) {
  __m128 t128     = _mm_loadu_ps(reinterpret_cast<const float *>(&T));
  __m128 v128     = _mm_loadu_ps(reinterpret_cast<const float *>(&v));
  __m128 q128cssc = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(t128), 0xeb));
  __m128 v128xxyy = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(v128), 0x50));
  __m128 m4       = _mm_mul_ps(q128cssc, v128xxyy);
  __m128 zero     = _mm_setzero_ps();
  __m128 p128xy00 = _mm_shuffle_ps(t128, zero, 0x4);
  __m128 temp     = _mm_add_ps(m4, p128xy00);
//  reportVal("t128", t128);
//  reportVal("v128", v128);
//  reportVal("q128cssc", q128cssc);
//  reportVal("v128xxyy", v128xxyy);
//  reportVal("m4", m4);
//  reportVal("p128xy00", p128xy00);
//  reportVal("temp", temp);
  float32 tempMem[4];
  b2Vec2  result;
  _mm_storeu_ps(tempMem, temp);
  result.x = tempMem[0] - tempMem[2];
  result.y = tempMem[1] + tempMem[3];
  return result;
}

inline b2Vec2 b2Mul(const b2Transform& T, const b2Vec2& v) {
  b2Vec2 result;
  result.x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
  result.y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;
  return result;
}

typedef b2Vec2 (*mulFunc)(const b2Transform& T, const b2Vec2& v);

int main(int argc, char **argv) {
  b2Transform t = {{1.0, 2.0}, {0.1, 0.2}};
  b2Vec2      r = {1.0, 2.0};
  mulFunc     mul;
  if (argc > 1) {
    printf("Using SIMD\n");
    mul = b2MulSimd;
  }
  else {
    printf("Using Scalar\n");
    mul = b2Mul;
  }
  b2Vec2 r1 = b2Mul(t, r);
  b2Vec2 r2 = b2MulSimd(t, r);
  if (r1.x != r2.x || r1.y != r2.y) {
    printf("Error: (%.2f, %.2f) != (%.2f, %.2f)\n", r1.x, r1.y, r2.x, r2.y);
    return 1;
  }
  for (int i = 0; i < 100000000; ++i) {
    r = mul(t, r);
  }
  printf("%f, %f\n", r.x, r.y);
  return 0;
}
