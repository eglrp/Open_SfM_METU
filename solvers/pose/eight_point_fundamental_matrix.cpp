

#include "Open_SfM_METU/solvers/pose/eight_point_fundamental_matrix.hpp"


#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/LU>



#include "Open_SfM_METU/solvers/pose_util.hpp"


namespace Open_SfM_METU {
namespace solvers {


using Eigen::JacobiSVD;
using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::Vector2d;
using Eigen::Vector3d;

bool NormalizedEightPointFundamentalMatrix(
    const std::vector<Vector2d>& image_1_points,
    const std::vector<Vector2d>& image_2_points,
    Matrix3d* fundamental_matrix) {
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

bool NormalizedEightPointFundamentalMatrixWithRadialDistortion(    
	const std::vector<Vector2d>& image_1_points,
    const std::vector<Vector2d>& image_2_points,
    Matrix3d* fundamental_matrix){

	  CHECK_EQ(image_1_points.size(), image_2_points.size());
	  CHECK_GE(image_1_points.size(), 8);

	  std::vector<Vector2d> norm_img1_points(image_1_points.size());
	  std::vector<Vector2d> norm_img2_points(image_2_points.size());

	  // Normalize the image points.
	  Matrix3d img1_norm_mat, img2_norm_mat;
	  NormalizeImagePoints(image_1_points, &norm_img1_points, &img1_norm_mat);
	  NormalizeImagePoints(image_2_points, &norm_img2_points, &img2_norm_mat);


	  // Constraint matrix D1
	  // Build the constraint matrix based on x2' * F * x1 = 0.
	  Matrix<double, Eigen::Dynamic, 9> constraint_matrix_D1(image_1_points.size(), 9);
	  for (int i = 0; i < image_1_points.size(); i++) {
	    constraint_matrix_D1.block<1, 3>(i, 0) = norm_img1_points[i].homogeneous();
	    constraint_matrix_D1.block<1, 3>(i, 0) *= norm_img2_points[i].x();
	    constraint_matrix_D1.block<1, 3>(i, 3) = norm_img1_points[i].homogeneous();
	    constraint_matrix_D1.block<1, 3>(i, 3) *= norm_img2_points[i].y();
	    constraint_matrix_D1.block<1, 3>(i, 6) = norm_img1_points[i].homogeneous();
	  }

	  // Constraint matrix D2
  	  Matrix<double, Eigen::Dynamic, 9> constraint_matrix_D2(image_1_points.size(), 9);

  	  Vector2d zero_vec;
  	  zero_vec<<0,0; 
  	  for (int i = 0; i < image_1_points.size(); i++) {

  	  	// r_prime_square
  	  	double r_square_image_point_2 = norm_img2_points[i].transpose() * norm_img2_points[i];
  	  	// r_square
  	  	double r_square_image_point_1 = norm_img1_points[i].transpose() * norm_img1_points[i];



  	  	constraint_matrix_D2.block<1, 2>(i, 0) = zero_vec;
  	  	constraint_matrix_D2(i, 2) = r_square_image_point_1 * norm_img2_points[i].x();;
  	  	constraint_matrix_D2.block<1, 2>(i, 3) = zero_vec;
  	  	constraint_matrix_D2(i, 5) = r_square_image_point_1 * norm_img2_points[i].y();;
  	  	constraint_matrix_D2.block<1, 3>(i, 6) = norm_img1_points[i].homogeneous();
  	  	constraint_matrix_D2.block<1, 3>(i, 6) *= r_square_image_point_2;
		constraint_matrix_D2(i, 8) += r_square_image_point_1;
  	  }

  	  

  	  /*

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

	  */

	  return true;

}

}
}