#ifndef CAMERA_UTILS_HPP
#define CAMERA_UTILS_HPP

#include <Eigen/Core>
#include <vector>

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

bool umdistortImagePoint(Eigen::Vector2d& inputCorrd, Eigen::Vector2d& outputCorrd);


} // camera
} // Open_SfM_METU



#endif
