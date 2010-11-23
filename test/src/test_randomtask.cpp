#include "Math/FastMath.h"
#include "harness_flight.hpp"

int main(int argc, char** argv) 
{
  // default arguments
  bearing_noise=0;
  target_noise=0.1;
  turn_speed=5.0;
  output_skip = 5;

  if (!parse_args(argc,argv)) {
    return 0;
  }

#define NUM_TESTS 2

  plan_tests(NUM_TESTS);

  for (int j=0; j<NUM_TESTS; j++) {
    unsigned i = rand()%NUM_WIND;

    if (j+1==NUM_TESTS) {
      verbose=1;
    }
    ok (test_flight_times(7,i), test_name("flight times",7,i),0);
  }
  return exit_status();
}
