#ifndef OPEN_SFM_METU_SOLVERS_POSE_UTIL_HPP
#define OPEN_SFM_METU_SOLVERS_POSE_UTIL_HPP



#include <Eigen/Core>
#include <vector>

namespace Open_SfM_METU {
namespace solvers {


class Camera;
struct FeatureCorrespondence;

// Calculates Sampson distance for two correspondances and an essential or
// fundamental matrix by eq. 11.9 in Hartley and Zisserman. For an E or F
// that is defined such that y^t * (E or F) * x = 0
double SquaredSampsonDistance(const Eigen::Matrix3d& F,
                              const Eigen::Vector2d& x,
                              const Eigen::Vector2d& y);

// Returns the cross product matrix of a vector: if cross_vec = [x y z] then
//                        [ 0  -z   y]
// cross product matrix = [ z   0  -y]
//                        [-y   x   0]
Eigen::Matrix3d CrossProductMatrix(const Eigen::Vector3d& cross_vec);

// Given a 2xN matrix image points. This functionnormalize the image pixels
// so that (0,0) point is at the image center. This is method 2
bool NormalizeImagePoints_M2(
    const std::vector<Eigen::Vector2d>& image_points,
    std::vector<Eigen::Vector2d>* normalized_image_points);

// Given a 2xN matrix of image points (of the form [x, y]), this method
// calculates the matrix that will shift the points so that the centroid is at
// the origin and the average distance from the centroid is sqrt(2). Returns the
// transformation matrix and the transformed points.
bool NormalizeImagePoints(
    const std::vector<Eigen::Vector2d>& image_points,
    std::vector<Eigen::Vector2d>* normalized_image_points,
    Eigen::Matrix3d* normalization_matrix);



// Projects a 3x3 matrix to the rotation matrix in SO3 space with the closest
// Frobenius norm. For a matrix with an SVD decomposition M = USV, the nearest
// rotation matrix is R = UV'.
Eigen::Matrix3d ProjectToRotationMatrix(const Eigen::Matrix3d& matrix);



}
}





#endif // OPEN_SFM_METU_SOLVERS_POSE_UTIL_HPP