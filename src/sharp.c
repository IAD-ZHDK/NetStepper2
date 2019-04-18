#include "sharp.h"

#include <math.h>
#include <stdint.h>

// Imported from https://github.com/DrGFreeman/SharpDistSensor.

double sharp_convert(double v) {
  // calculate distance from polynomial fit function
  uint16_t dist = 0;
  dist += -2.037E-12 * pow(v, 5);
  dist += 1.167E-8 * pow(v, 4);
  dist += -2.251E-5 * pow(v, 3);
  dist += 2.023E-2 * pow(v, 2);
  dist += -9.005 * v;
  dist += 1734;

  // TODO: Fix this.

  // apply drift?
  dist *= 0.6;

  return dist;
}
