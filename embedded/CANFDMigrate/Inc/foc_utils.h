#pragma once

#define _PI   3.14159265358979323846f
#define _2PI  6.28318530717958647692f    // 2*π

inline float _normalizeAngle(float ang) {
  while (ang < 0)      ang += _2PI;
  while (ang >= _2PI)  ang -= _2PI;
  return ang;
}
