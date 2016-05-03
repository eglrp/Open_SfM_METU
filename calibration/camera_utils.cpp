



#include "Open_SfM_METU/calibration/camera_utils.hpp"
#include <math.h>

namespace Open_SfM_METU {
namespace camera {

#define _USE_MATH_DEFINES


double afov2focal(double AFOV, double h){

	return ((h / 2.0) / (tan((AFOV / 2.0) * M_PI / 180)));
}

double focal2afov(double focal, double h){

	return (2.0 * atan(h / (2.0 * focal)) * 180 / M_PI);

}


} // camera
} // Open_SfM_METU