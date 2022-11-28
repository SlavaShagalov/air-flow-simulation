#include "utils.h"

bool doubleEq(const double a, const double b) { return fabs(a - b) < EPS; }

int sign(const float x) {
  if (fabs(x) < EPS)
    return 0;
  if (x < 0)
    return -1;
  else
    return 1;
}

int sign(const int x) {
  if (x < 0)
    return -1;
  else if (x > 0)
    return 1;

  return 0;
}
