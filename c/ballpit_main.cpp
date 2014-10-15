#include <stdio.h>

#include "ballpit.h"

int main(int argc, char** argv) {
  result_t result = ballpit(argc, argv);
  printf("Benchmark complete.\n  ms/frame: %f 5th %%ile: %f 95th %%ile: %f\n", result.mean, result.pc_5th, result.pc_95th);
  return 0;
}
