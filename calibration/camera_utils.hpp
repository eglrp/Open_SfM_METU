#ifndef CAMERA_UTILS_HPP
#define CAMERA_UTILS_HPP

#include <Eigen/Core>
#include <vector>

#include <glog/logging.h>

#include "Open_SfM_METU/image/image.hpp"

namespace Open_SfM_METU {
namespace camera {


// AFOV  : Angle Field of View in degree
// h 	 ; Horizontal dimension of lense in pixel
double afov2focal(double AFOV, double h);

// focal : Camera focal length in pixel
// h 	 ; Horizontal dimension of lense in pixel
double focal2afov(double focal, double h);

// Undistort image by using given focal length and radial distortion parameter
// according to the distoriton model in 
// "Simultaneous linear estimation of multiple view geometry and lens distortion"

// The given coordinates are assummed to be normalized coordinates. 

bool undistortImage(image::Image<unsigned char>& inputImage, image::Image<unsigned char>& outputImage, Eigen::Matrix3d& K_mat, double lambda);

bool undistortImagePoint(Eigen::Vector2d& norm_inputPoint2d, Eigen::Vector2d& norm_outputPoint2d, double lambda);


} // camera
} // Open_SfM_METU



#endif
