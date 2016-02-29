


#include <iostream>

#include "Open_SfM_METU/solvers/pose_util.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <glog/logging.h>

#include "Open_SfM_METU/solvers/random.hpp"
#include "Open_SfM_METU/matching/feature_correspondence.hpp"


namespace Open_SfM_METU {
namespace solvers {

using Eigen::Map;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;


// For an E or F that is defined such that y^t * E * x = 0
double SquaredSampsonDistance(const Matrix3d& F,
                              const Vector2d& x,
                              const Vector2d& y) {
  const Vector3d epiline_x = F * x.homogeneous();
  const double numerator_sqrt = y.homogeneous().dot(epiline_x);
  const Vector4d denominator(y.homogeneous().dot(F.col(0)),
                             y.homogeneous().dot(F.col(1)),
                             epiline_x[0],
                             epiline_x[1]);

  // Finally, return the complete Sampson distance.
  return numerator_sqrt * numerator_sqrt / denominator.squaredNorm();
}

Eigen::Matrix3d CrossProductMatrix(const Vector3d& cross_vec) {
  Matrix3d cross;
  cross << 0.0, -cross_vec.z(), cross_vec.y(),
      cross_vec.z(), 0.0, -cross_vec.x(),
      -cross_vec.y(), cross_vec.x(), 0.0;
  return cross;
}


// Given a 2xN matrix image points. This functionnormalize the image pixels
// so that (0,0) point is at the image center. This is method 2.
bool NormalizeImagePoints_M2(
    const std::vector<Eigen::Vector2d>& image_points,
    std::vector<Eigen::Vector2d>* normalized_image_points){

    normalized_image_points->resize(image_points.size());

    Eigen::Map<const Matrix<double, 2, Eigen::Dynamic> > image_points_mat( image_points[0].data(), 2, image_points.size());

    std::cout << "point check " << image_points[image_points.size()-2] << std::endl;
    std::cout << "point check " << image_points[image_points.size()-1] << std::endl;
    std::cout << "Mat check " << std::endl;
    std::cout <<  image_points_mat << std::endl;

    Eigen::Matrix<double, 2, 1> m_1;
      m_1(0,0) = 960;
      m_1(1,0) = 540;
    Eigen::Matrix<double, 2, 1> m_2;
      m_2(0,0) = 1.0f/1920;
      m_2(1,0) = 1.0f/1080;

    for(int i = 0; i < image_points.size(); i++){

      //std::cout << "col mat " << image_points_mat.col(i) << std::endl;
      //std::cout << "diff mat " <<  m_1 << std::endl;
      //std::cout << "res mat " << image_points_mat.col(i) - m_1 << std::endl;
      Eigen::Matrix<double, 2, 1> t_M;
      t_M =  image_points_mat.col(i) - m_1;
      t_M = (t_M.array() * m_2.array()).matrix();
      //image_points_mat.col(i) = image_points_mat.col(i) - m_1;
      //image_points_mat.col(i) = (image_points_mat.col(i).array() * temp_vec_2.array()).matrix();

    }


    return true;

}


// Computes the normalization matrix transformation that centers image points
// around the origin with an average distance of sqrt(2) to the centroid.
// Returns the transformation matrix and the transformed points. This assumes
// that no points are at infinity.
bool NormalizeImagePoints(
    const std::vector<Vector2d>& image_points,
    std::vector<Vector2d>* normalized_image_points,
    Matrix3d* normalization_matrix) {
  Eigen::Map<const Matrix<double, 2, Eigen::Dynamic> > image_points_mat(
      image_points[0].data(), 2, image_points.size());

  std::cout <<  "image_points_mat" << std::endl;
  std::cout <<  image_points_mat << std::endl;


  // Allocate the output vector and map an Eigen object to the underlying data
  // for efficient calculations.
  normalized_image_points->resize(image_points.size());
  Eigen::Map<Matrix<double, 2, Eigen::Dynamic> >
      normalized_image_points_mat((*normalized_image_points)[0].data(), 2,
                                  image_points.size());


  // Compute centroid.
  const Vector2d centroid(image_points_mat.rowwise().mean());

  //std::cout << "centroid = " << centroid << std::endl;

  // Calculate average RMS distance to centroid.
  const double rms_mean_dist =
      sqrt((image_points_mat.colwise() - centroid).squaredNorm() /
           image_points.size());

  // Create normalization matrix.
  const double norm_factor = sqrt(2.0) / rms_mean_dist;
  *normalization_matrix << norm_factor, 0, -1.0 * norm_factor* centroid.x(),
      0, norm_factor, -1.0 * norm_factor * centroid.y(),
      0, 0, 1;

  // Normalize image points.
  const Matrix<double, 3, Eigen::Dynamic> normalized_homog_points =
      (*normalization_matrix) * image_points_mat.colwise().homogeneous();
  normalized_image_points_mat = normalized_homog_points.colwise().hnormalized();

  return true;
}

// Projects a 3x3 matrix to the rotation matrix in SO3 space with the closest
// Frobenius norm. For a matrix with an SVD decomposition M = USV, the nearest
// rotation matrix is R = UV'.
Matrix3d ProjectToRotationMatrix(const Matrix3d& matrix) {
  Eigen::JacobiSVD<Matrix3d> svd(matrix,
                                 Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix3d rotation_mat = svd.matrixU() * (svd.matrixV().transpose());

  // The above projection will give a matrix with a determinant +1 or -1. Valid
  // rotation matrices have a determinant of +1.
  if (rotation_mat.determinant() < 0) {
    rotation_mat *= -1.0;
  }

  return rotation_mat;
}


}
}


