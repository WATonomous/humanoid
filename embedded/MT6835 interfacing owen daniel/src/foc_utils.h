#ifndef ANGLE_UTILS_H
#define ANGLE_UTILS_H

#include <math.h>

#define PI     3.14159265358979323846f
#define TWO_PI 6.28318530717958647692f

static inline float normalize_angle(float ang) {
    while (ang < 0.0f)      ang += TWO_PI;
    while (ang >= TWO_PI)   ang -= TWO_PI;
    return ang;
}

#endif // ANGLE_UTILS_H
