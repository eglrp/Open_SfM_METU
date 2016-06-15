#ifndef OPEN_SFM_METU_SOLVERS_POSE_EIGHT_POINT_HPP
#define OPEN_SFM_METU_SOLVERS_POSE_EIGHT_POINT_HPP


#include <Eigen/Core>
#include <vector>

#include <iostream>

#include <glog/logging.h>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>

#include <iostream>
#include <math.h> 
#include <memory>
#include <algorithm>
#include <functional>

#include "Open_SfM_METU/solvers/pose_util.hpp"
#include "Open_SfM_METU/solvers/random.hpp"

#include "Open_SfM_METU/calibration/camera_utils.hpp"


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


using Eigen::JacobiSVD;
using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::Vector2d;
using Eigen::Vector3d;

inline bool NormalizedEightPointFundamentalMatrix_v2(
    const std::vector<Eigen::Vector2d>& image_1_points,
    const std::vector<Eigen::Vector2d>& image_2_points,
    Eigen::Matrix3d* fundamental_matrix){



  CHECK_EQ(image_1_points.size(), image_2_points.size());
  CHECK_GE(image_1_points.size(), 8);

  std::vector<Vector2d> norm_img1_points(image_1_points.size());
  std::vector<Vector2d> norm_img2_points(image_2_points.size());

  // Normalize the image points.
  Matrix3d img1_norm_mat, img2_norm_mat;
  NormalizeImagePoints(image_1_points, &norm_img1_points, &img1_norm_mat);
  NormalizeImagePoints(image_2_points, &norm_img2_points, &img2_norm_mat);

  // Build the constraint matrix based on x2' * F * x1 = 0.
  Matrix<double, Eigen::Dynamic, 9> constraint_matrix(image_1_points.size(), 9);
  for (int i = 0; i < image_1_points.size(); i++) {
    constraint_matrix.block<1, 3>(i, 0) = norm_img1_points[i].homogeneous();
    constraint_matrix.block<1, 3>(i, 0) *= norm_img2_points[i].x();
    constraint_matrix.block<1, 3>(i, 3) = norm_img1_points[i].homogeneous();
    constraint_matrix.block<1, 3>(i, 3) *= norm_img2_points[i].y();
    constraint_matrix.block<1, 3>(i, 6) = norm_img1_points[i].homogeneous();
  }

  // Solve the constraint equation for F from nullspace extraction.
  // An LU decomposition is efficient for the minimally constrained case.
  // Otherwise, use an SVD.
  Matrix<double, 9, 1> normalized_fvector;
  if (image_1_points.size() == 8) {
    const auto lu_decomposition = constraint_matrix.fullPivLu();
    if (lu_decomposition.dimensionOfKernel() != 1) {
      return false;
    }
    // kernel of matrix is also called NULL-SPACE. Columns of returned matrix will form a basis of the kernel
    normalized_fvector = lu_decomposition.kernel();
  } else {
    JacobiSVD<Matrix<double, Eigen::Dynamic, 9> > cmatrix_svd(
       constraint_matrix, Eigen::ComputeFullV);
    normalized_fvector = cmatrix_svd.matrixV().col(8);
  }

  // NOTE: This is the transpose of a valid fundamental matrix! We implement a
  // "lazy" transpose and defer it to the SVD a few lines below.
  Eigen::Map<const Matrix3d> normalized_fmatrix(normalized_fvector.data());

  // Find the closest singular matrix to F under frobenius norm. We can compute
  // this matrix with SVD.
  JacobiSVD<Matrix3d> fmatrix_svd(normalized_fmatrix.transpose(),
                                  Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vector3d singular_values = fmatrix_svd.singularValues();
  singular_values[2] = 0.0;
  *fundamental_matrix = fmatrix_svd.matrixU() * singular_values.asDiagonal() *
                        fmatrix_svd.matrixV().transpose();

  // Correct for the point normalization.
  *fundamental_matrix =
      img2_norm_mat.transpose() * (*fundamental_matrix) * img1_norm_mat;

  return true;


}

inline bool NormalizedN_pointFundamentalMatrix_v2(
    const std::vector<Eigen::Vector2d>& image_1_points,
    const std::vector<Eigen::Vector2d>& image_2_points,
    Eigen::Matrix3d* fundamental_matrix){


  CHECK_EQ(image_1_points.size(), image_2_points.size());

  std::vector<Vector2d> norm_img1_points(image_1_points.size());
  std::vector<Vector2d> norm_img2_points(image_2_points.size());

  // Normalize the image points.
  Matrix3d img1_norm_mat, img2_norm_mat;
  NormalizeImagePoints(image_1_points, &norm_img1_points, &img1_norm_mat);
  NormalizeImagePoints(image_2_points, &norm_img2_points, &img2_norm_mat);

  // Build the constraint matrix based on x2' * F * x1 = 0.
  Matrix<double, Eigen::Dynamic, 9> constraint_matrix(image_1_points.size(), 9);
  for (int i = 0; i < image_1_points.size(); i++) {
    constraint_matrix.block<1, 3>(i, 0) = norm_img1_points[i].homogeneous();
    constraint_matrix.block<1, 3>(i, 0) *= norm_img2_points[i].x();
    constraint_matrix.block<1, 3>(i, 3) = norm_img1_points[i].homogeneous();
    constraint_matrix.block<1, 3>(i, 3) *= norm_img2_points[i].y();
    constraint_matrix.block<1, 3>(i, 6) = norm_img1_points[i].homogeneous();
  }

  // Solve the constraint equation for F from nullspace extraction.
  // An LU decomposition is efficient for the minimally constrained case.
  // Otherwise, use an SVD.
  Matrix<double, 9, 1> normalized_fvector;
  if (image_1_points.size() == 8) {
    const auto lu_decomposition = constraint_matrix.fullPivLu();
    if (lu_decomposition.dimensionOfKernel() != 1) {
      return false;
    }
    // kernel of matrix is also called NULL-SPACE. Columns of returned matrix will form a basis of the kernel
    normalized_fvector = lu_decomposition.kernel();
  } else {
    JacobiSVD<Matrix<double, Eigen::Dynamic, 9> > cmatrix_svd(
       constraint_matrix, Eigen::ComputeFullV);
    normalized_fvector = cmatrix_svd.matrixV().col(8);

    //std::cout << "Constrained matrix is " << std::endl;
    //std::cout << constraint_matrix << std::endl;
    std::cout << "normalized vector of F" << std::endl;
    std::cout << normalized_fvector << std::endl;

  }

  // NOTE: This is the transpose of a valid fundamental matrix! We implement a
  // "lazy" transpose and defer it to the SVD a few lines below.
  Eigen::Map<const Matrix3d> normalized_fmatrix(normalized_fvector.data());

  // Find the closest singular matrix to F under frobenius norm. We can compute
  // this matrix with SVD.
  JacobiSVD<Matrix3d> fmatrix_svd(normalized_fmatrix.transpose(),
                                  Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vector3d singular_values = fmatrix_svd.singularValues();
  singular_values[2] = 0.0;
  *fundamental_matrix = fmatrix_svd.matrixU() * singular_values.asDiagonal() *
                        fmatrix_svd.matrixV().transpose();

  std::cout << "result F" << std::endl;
    std::cout << img2_norm_mat.transpose() * (*fundamental_matrix) * img1_norm_mat << std::endl;                        

  // Correct for the point normalization.
  *fundamental_matrix =
      img2_norm_mat.transpose() * (*fundamental_matrix) * img1_norm_mat;

  return true;


}




}
}




#endif // OPEN_SFM_METU_SOLVERS_POSE_EIGHT_POINT_HPP