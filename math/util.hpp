

#ifndef OPEN_SFM_METU_MATH_UTIL_H_
#define OPEN_SFM_METU_MATH_UTIL_H_

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

namespace Open_SfM_METU {
namespace math {

static const double kRadToDeg = 180.0 / M_PI;
static const double kDegToRad = M_PI / 180.0;

inline double RadToDeg(double angle_radians) {
  return angle_radians * kRadToDeg;
}

inline double DegToRad(double angle_degrees) {
  return angle_degrees * kDegToRad;
}

inline double Clamp(const double val, const double min, const double max) {
  return std::max(min, std::min(val, max));
}

}  // math
}  // Open_SfM_METU

#endif  // OPEN_SFM_METU_MATH_UTIL_H_
