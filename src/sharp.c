#include <art32/numbers.h>
#include <art32/smooth.h>
#include <math.h>

// Imported from https://github.com/DrGFreeman/SharpDistSensor.

a32_smooth_t* smooth;

void sharp_init() {
  // initialize smooth
  smooth = a32_smooth_new(16);
}

double sharp_convert(double v) {
  // constrain sensor value
  v = a32_constrain_d(v, 30, 875);

  // Calculate distance from polynomial fit function
  double dist = 0;
  dist += -2.037E-12 * pow(v, 5);
  dist += 1.167E-8 * pow(v, 4);
  dist += -2.251E-5 * pow(v, 3);
  dist += 2.023E-2 * pow(v, 2);
  dist += -9.005 * v;
  dist += 1734;

  // return smoothed value
  return a32_smooth_update(smooth, dist);
}
