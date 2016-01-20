#ifndef OPEN_SFM_METU_SOLVERS_POSE_EIGHT_POINT_HPP
#define OPEN_SFM_METU_SOLVERS_POSE_EIGHT_POINT_HPP


#include <Eigen/Core>
#include <vector>

namespace Open_SfM_METU {
namespace solvers {


// Computes the Fundamental Matrix
// (http://en.wikipedia.org/wiki/Fundamental_matrix_(computer_vision) ) from 8
// or more image correspondences according to the normalized 8 point algorithm
// (Hartley and Zisserman alg 11.1 page 282). Image points are first normalized
// by a translation and scale, and the fundamental matrix is computed from the
// singular vector corresponding to the smallest singular vector of the stacked
// epipolar constraints. The estimated fundamental matrix is the computed
// fundamental matrix with the normalization transformation undone.
//
// Params:
//   image_1_points: image points from one image (8 or more).
//   image_2_points: image points from a second image (8 or more).
//   fundamental_matrix: the estimated fundamental matrix such that
//     x2^t * F * x1 = 0 for points x1 in image_1_points and x2 in
//     image_2_points.


bool NormalizedEightPointFundamentalMatrix(
    const std::vector<Eigen::Vector2d>& image_1_points,
    const std::vector<Eigen::Vector2d>& image_2_points,
    Eigen::Matrix3d* fundamental_matrix);

bool NormalizedEightPointFundamentalMatrixWithRadialDistortion(
    const std::vector<Eigen::Vector2d>& image_1_points,
    const std::vector<Eigen::Vector2d>& image_2_points,
    Eigen::Matrix3d* fundamental_matrix);


}
}




#endif // OPEN_SFM_METU_SOLVERS_POSE_EIGHT_POINT_HPP