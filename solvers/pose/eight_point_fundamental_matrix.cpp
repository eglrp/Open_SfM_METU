

#include "Open_SfM_METU/solvers/pose/eight_point_fundamental_matrix.hpp"


#include <glog/logging.h>
#include <Eigen/Core>
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


// The polynomial aigen value problem is stated as
// (A_0 + λ*A_1 + ... + λ^(p) * A_p)
// _input_matrices_vec: they are in order as began from A_0 to A_p. 

bool polyEig(std::vector<Eigen::MatrixXd> _input_matrices_vec, 
												int n_value, 
												int p_value, 
												Eigen::VectorXcd* E_result, 
												Eigen::MatrixXcd* X_result){

	
	
	/*	
	std::cout << "matrix_D1 is in polyeig" << std::endl;
	std::cout << _input_matrices_vec[0] << std::endl;


	std::cout << "matrix_D2 is in polyeig" << std::endl;
	std::cout << _input_matrices_vec[1] << std::endl;


	std::cout << "matrix_D3 is in polyeig" << std::endl;
	std::cout << _input_matrices_vec[2] << std::endl;
	*/
	
	


	//Matrix<double, Eigen::Dynamic, n_value * p_value> A_mat(n_value * p_value, n_value * p_value);
	Eigen::MatrixXd A_mat(n_value * p_value, n_value * p_value);
	A_mat = Eigen::MatrixXd::Identity(n_value * p_value, n_value * p_value);

	

	// We used differnet block operation function here since matrix dimension is dynamic here. 

	A_mat.block(0,0,n_value,n_value) = _input_matrices_vec[0];


	Eigen::MatrixXd B_mat((n_value * (p_value - 1) + n_value), (n_value * (p_value - 1) + n_value));

	B_mat = Eigen::MatrixXd::Zero((n_value * (p_value - 1) + n_value), (n_value * (p_value - 1) + n_value));


	if(p_value > 0){

		B_mat.bottomLeftCorner(n_value * (p_value - 1), n_value * (p_value - 1)) = Eigen::MatrixXd::Identity(n_value * (p_value - 1), n_value * (p_value - 1));

		for(int k = 0; k < p_value; k++){

			B_mat.block(0, (k * n_value), n_value, n_value) = -(_input_matrices_vec[k+1]);

		}

	}
	else{

		std::cout << "error occured " << std::endl;
		return false; 
	}

	/*

	std::cout << "A matrix is " << std::endl;
	std::cout << A_mat << std::endl;

	std::cout << "B matrix is " << std::endl;
	std::cout << B_mat << std::endl;

	*/

	/*

	// Here we convert A_mat and B_mat to complex matrices

	Eigen::MatrixXcd A_mat_c(A_mat.rows(),A_mat.cols()), B_mat_c(B_mat.rows(),B_mat.cols());
	A_mat_c = Eigen::MatrixXcd::Identity(A_mat.rows(),A_mat.cols());
	B_mat_c = Eigen::MatrixXcd::Identity(B_mat.rows(),B_mat.cols());

	for(size_t j=0; j<A_mat.cols(); ++j) {
	    for(size_t i=0; i<A_mat.rows(); ++i) {
	        //A_mat_c(i,j) = std::complex<double>(A_mat(i,j), 0);
	    	A_mat_c.real()(i,j) = A_mat(i,j);
	    	A_mat_c.imag()(i,j) = 0;
	    }
	}

	for(size_t j=0; j<B_mat.cols(); ++j) {
	    for(size_t i=0; i<B_mat.rows(); ++i) {
	        B_mat_c(i,j) = std::complex<double>(B_mat(i,j), 0);
	    }
	}
	

	std::cout << "Complex A mat " << std::endl;
	std::cout << A_mat_c << std::endl;

	std::cout << "Complex B mat " << std::endl;
	std::cout << B_mat_c << std::endl;

	*/	

	Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> es_temp;
	//Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> es_temp;

	//Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXcd> es_temp(A_mat_c,B_mat_c);

	es_temp.compute(A_mat,B_mat,true);



	

	//Eigen::MatrixXd temp_mat(n_value * p_value,n_value * p_value); 
	//temp_mat = B_mat.inverse() * A_mat;
	//temp_mat = Eigen::MatrixXd::Random(n_value * p_value,n_value * p_value);
	//Eigen::EigenSolver<Eigen::MatrixXd> es_temp(temp_mat,true);

	

	/*

	Eigen::MatrixXd temp_mat_real(n_value * p_value,n_value * p_value); 
	temp_mat_real = B_mat.inverse() * A_mat;



	Eigen::MatrixXcd temp_mat(n_value * p_value,n_value * p_value); 
	//temp_mat = B_mat_c.inverse() * A_mat_c;


	for(size_t j=0; j<temp_mat_real.cols(); ++j) {
	    for(size_t i=0; i<temp_mat_real.rows(); ++i) {
	        temp_mat(i,j) = std::complex<double>(temp_mat_real(i,j), 0);
	    }
	}



	Eigen::ComplexEigenSolver<Eigen::MatrixXcd> es_temp;
	es_temp.compute(temp_mat,true);


	std::cout << "Complex temp mat = " << std::endl;
	std::cout << temp_mat << std::endl;

	*/

	


	Eigen::MatrixXcd X_eigen_vec(n_value * p_value, n_value * p_value);
	X_eigen_vec = es_temp.eigenvectors();

	// oncelikle eigen vectorler real mi geliyor bakalim



	// Burada eigen vector mat mi vec mi ne donuyor ???
	// Real mi complex mi donuyor
	

	Eigen::VectorXcd E_eigen_val(n_value * p_value,1);
	E_eigen_val = es_temp.eigenvalues();

	

	/*
	

	std::cout << "Eigen Values of system with A,B " << std::endl;
	std::cout << es_temp.eigenvalues() << std::endl;
	//std::cout << E_eigen_val << std::endl;

	std::cout << "Eigen Vectors of system with A,B " << std::endl;
	std::cout << es_temp.eigenvectors() << std::endl;
	//std::cout << X_eigen_vec << std::endl;

	*/
	
	// X ve E matrix type degisimini yansitalim asagiya. 

	

	

	if(p_value >= 2){

		// For each eigenvalue, extract the eigenvector from whichever portion
   		// of the big eigenvector matrix X gives the smallest normalized residual.


   		Eigen::MatrixXcd V_temp = Eigen::MatrixXcd::Zero(n_value, p_value);

   		
		for(int j = 0; j < (n_value * p_value); j++ ){

			for(int ind = 0; ind < p_value; ind++ ){
				V_temp.block(0,ind,n_value,1) = X_eigen_vec.col(j).block(ind,0,n_value,1);
			}	

			Eigen::MatrixXcd R_temp (n_value,n_value);

			Eigen::MatrixXcd input_temp (n_value,n_value);

			
			// an input matrix is converted to the complex form

			for(size_t j=0; j<_input_matrices_vec[p_value].cols(); ++j) {
			    for(size_t i=0; i<_input_matrices_vec[p_value].rows(); ++i) {
			        //A_mat_c(i,j) = std::complex<double>(A_mat(i,j), 0);
			    	input_temp.real()(i,j) = _input_matrices_vec[p_value](i,j);
			    	input_temp.imag()(i,j) = 0;
			    }
			}

			R_temp = input_temp;

			

			if(!std::isnan(E_eigen_val(j).imag()) && !std::isnan(E_eigen_val(j).real())){

				for(int t = p_value ; t > 0; t--){

					Eigen::MatrixXcd input_temp_2 (n_value,n_value);


					for(size_t j=0; j<_input_matrices_vec[(t-1)].cols(); ++j) {
					    for(size_t i=0; i<_input_matrices_vec[(t-1)].rows(); ++i) {
					        //A_mat_c(i,j) = std::complex<double>(A_mat(i,j), 0);
					    	input_temp_2.real()(i,j) = _input_matrices_vec[(t-1)](i,j);
					    	input_temp_2.imag()(i,j) = 0;
					    }
					}

					//R_temp = _input_matrices_vec[(t-1)] + E_eigen_val(j) * R_temp;
					R_temp = input_temp_2 + E_eigen_val(j) * R_temp;
				}

			}


			R_temp = R_temp * V_temp;

			Eigen::MatrixXd res; 
			res = ((R_temp.cwiseAbs()).colwise().sum()).cwiseQuotient((V_temp.cwiseAbs()).colwise().sum());
	

			Eigen::MatrixXf::Index minRow, minCol;
  			float min = res.minCoeff(&minRow, &minCol);

  			X_eigen_vec.block( 0, j, n_value, 1) = (V_temp.col(minCol))/((V_temp.col(minCol)).norm());  // Eigenvector with unit 2-norm.

			

		}

		
		X_eigen_vec = X_eigen_vec.block(0,0,n_value,X_eigen_vec.cols());
		

	}



	*E_result = E_eigen_val;
	*X_result = X_eigen_vec;

	
	

	return true;
}


std::complex<double> eliminateMat(std::complex<double> x){

	std::cout << "inside function " << std::endl;

	if(x.imag() == 0){
		if(std::isnan(x.real()) || std::isinf(x.real())){

			return std::complex<double>(0, 0);
			//return 0;

		}
		else{

			return x;

		}
	}
	else{

		return std::complex<double>(0, 0);
		//return 0;
	}

}

bool NormalizedEightPointFundamentalMatrixWithRadialDistortion(    
	const std::vector<Vector2d>& image_1_points,
    const std::vector<Vector2d>& image_2_points,
    Matrix3d* fundamental_matrix,
    std::vector<double>& lambdaValues){

	  CHECK_EQ(image_1_points.size(), image_2_points.size());
	  CHECK_GE(image_1_points.size(), 9);

	  std::vector<Vector2d> norm_img1_points(image_1_points.size());
	  std::vector<Vector2d> norm_img2_points(image_2_points.size());

	  // Normalize the image points.
	  Matrix3d img1_norm_mat, img2_norm_mat;
	  NormalizeImagePoints(image_1_points, &norm_img1_points, &img1_norm_mat);
	  NormalizeImagePoints(image_2_points, &norm_img2_points, &img2_norm_mat);


	  // Constraint matrix D1
	  // Build the constraint matrix based on x2' * F * x1 = 0.
	  
	  //Matrix<double, Eigen::Dynamic, 9> constraint_matrix_D1(image_1_points.size(), 9);

	  /*
	  std::cout << "image point size in function " << image_1_points.size() << std::endl ;
	  std::cout << "image points in function " << std::endl;
	  std::cout << image_1_points[0].homogeneous() << " and " << image_1_points[1].homogeneous() << std::endl ;
	  std::cout << "norm image point in function " << std::endl;
	  std::cout << norm_img1_points[0].homogeneous() << " and " << norm_img1_points[1].homogeneous() << std::endl ;
	
	  */

	  //Eigen::MatrixXd constraint_matrix_D1(image_1_points.size(), 9);
	  Matrix<double, Eigen::Dynamic, 9> constraint_matrix_D1(image_1_points.size(), 9);
	  constraint_matrix_D1 = Eigen::MatrixXd::Zero(image_1_points.size(), 9);


	  for (int i = 0; i < image_1_points.size(); i++) {
	    
	    
	    constraint_matrix_D1.block<1, 3>(i, 0) = norm_img1_points[i].homogeneous();
	    constraint_matrix_D1.block<1, 3>(i, 0) *= norm_img2_points[i].x();
	    constraint_matrix_D1.block<1, 3>(i, 3) = norm_img1_points[i].homogeneous();
	    constraint_matrix_D1.block<1, 3>(i, 3) *= norm_img2_points[i].y();
	    constraint_matrix_D1.block<1, 3>(i, 6) = norm_img1_points[i].homogeneous();
		

		
	  }



	  // Constraint matrix D2
  	  Matrix<double, Eigen::Dynamic, 9> constraint_matrix_D2(image_1_points.size(), 9);
  	  constraint_matrix_D2 = Eigen::MatrixXd::Zero(image_1_points.size(), 9);


  	  Eigen::Vector2d zero_vec;
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



  	  // Constraint matrix D3
  	  Matrix<double, Eigen::Dynamic, 9> constraint_matrix_D3(image_1_points.size(), 9);
  	  constraint_matrix_D3 = Eigen::MatrixXd::Zero(image_1_points.size(), 9);



  	  Eigen::VectorXd zero_vec_2(8);
  	  zero_vec_2<<0,0,0,0,0,0,0,0; 
  	  
  	  for (int i = 0; i < image_1_points.size(); i++) {

  	  	// r_prime_square
  	  	double r_square_image_point_2 = norm_img2_points[i].transpose() * norm_img2_points[i];
  	  	// r_square
  	  	double r_square_image_point_1 = norm_img1_points[i].transpose() * norm_img1_points[i];

  	  	constraint_matrix_D3.block<1, 8>(i, 0) = zero_vec_2;
  	  	constraint_matrix_D3(i, 8) = r_square_image_point_2 * r_square_image_point_1;


  	  }



  	  /*
  	  
  	  // Solve QEP - Algorithm 1

  	  

  	  // This is a standard eigen value problem now which is converted from 
  	  // first QEP and then generalized eigen value problem. 
  	  Eigen::Matrix<double, Eigen::Dynamic, 18> sta_EVP(2*image_1_points.size(), 18);
  	  
  	  
	  constraint_matrix_D1 = constraint_matrix_D1.transpose() * constraint_matrix_D1;
	  constraint_matrix_D2 = constraint_matrix_D1.transpose() * constraint_matrix_D2;
  	  constraint_matrix_D3 = constraint_matrix_D1.transpose() * constraint_matrix_D3;

  	  sta_EVP.block<9,9>(0,0) = Eigen::MatrixXd::Zero(9,9);
  	  sta_EVP.block<9,9>(0,9) = Eigen::MatrixXd::Identity(9,9);
  	  

  	  sta_EVP.block<9,9>(9,0) = -(constraint_matrix_D3.inverse() * constraint_matrix_D1);
  	  sta_EVP.block<9,9>(9,9) = -(constraint_matrix_D3.inverse() * constraint_matrix_D2);

  	  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(sta_EVP);

  	  // std::cout << "Inverse D3 is  " << constraint_matrix_D3.transpose() *  constraint_matrix_D3<< std::endl;

	  
  	  
  	  std::cout << "D1 matrix is " << std::endl;
  	  std::cout << constraint_matrix_D1 << std::endl;
  	  std::cout << "D2 matrix is " << std::endl;
  	  std::cout << constraint_matrix_D2 << std::endl;
  	  std::cout << "D3 matrix is " << std::endl;
  	  std::cout << constraint_matrix_D3 << std::endl;

  	  std::cout << "The eigenvalues of sta_EVP are:" << es.eigenvalues() << std::endl;

	  */

	  // Solve QEP - Algorithm 2
  	  
  	  // PLEASE REMEMBE TO CHECK IF WE NEED TO USE ALIGNMENT OF EIGEN LIBRARY SINCE WE ARE USING VECTOR AND EIGEN MATRIX


	  constraint_matrix_D1 = constraint_matrix_D1.transpose() * constraint_matrix_D1;
	  constraint_matrix_D2 = constraint_matrix_D1.transpose() * constraint_matrix_D2;
  	  constraint_matrix_D3 = constraint_matrix_D1.transpose() * constraint_matrix_D3;

  	  /*

  	  std::cout << "D1 matrix is " << std::endl;
  	  std::cout << constraint_matrix_D1 << std::endl;
  	  std::cout << "D2 matrix is " << std::endl;
  	  std::cout << constraint_matrix_D2 << std::endl;
  	  std::cout << "D3 matrix is " << std::endl;
  	  std::cout << constraint_matrix_D3 << std::endl;

	  */
	  

  	  std::vector<Eigen::MatrixXd> input_matrices_vec;
  	  input_matrices_vec.push_back(constraint_matrix_D1);
  	  input_matrices_vec.push_back(constraint_matrix_D2);
  	  input_matrices_vec.push_back(constraint_matrix_D3);

  	  Eigen::VectorXcd E_result;
  	  Eigen::MatrixXcd X_result;
  	  
  	  int n_value =  constraint_matrix_D1.rows();
  	  int p_value =  input_matrices_vec.size() - 1;


  	  //std::cout << "size check " << image_1_points.size() << std::endl;

  	  if(polyEig(input_matrices_vec, n_value, p_value, &E_result, &X_result)){

  	  	

  	  	//std::cout << "polyeig is calculated" << std::endl;

		/*  	  	
  	  	std::cout << "The eigenvalues of sta_EVP are:" << std::endl;
  	  	std::cout << E_result<< std::endl;

  	  	std::cout << "The eigenvectors of sta_EVP are:" << std::endl;
  	  	std::cout << X_result<< std::endl;
		*/
		

  	  	// unaryExp ile hem real olan eigenvaluelari bulup onlarin da nan ve inf olanlarini atabilir miyiz. 
		// ayni zamanda buradaki fonksiyon complex alabilir mi ki deger 
		// once bu entryleri sifira cekip index bulacagiz. 
		

  	  
		//E_result.unaryExpr(std::ptr_fun(eliminateMat));
		//Eliminated_E_result.unaryExpr(std::ptr_fun(eliminateMat));

		Eigen::VectorXd real_E_result(9);
		real_E_result << 0,0,0,0,0,0,0,0,0;

		//std::cout << "real E_result " << real_E_result << std::endl;

  	  	for(int i = 0; i < 9 ; ++i){

  	  		if(E_result(i).imag() == 0){

  	  			
  	  			if(!std::isnan(E_result(i).real()) && !std::isinf(E_result(i).real())){

  	  				real_E_result(i) = E_result(i).real();

  	  				lambdaValues.push_back(real_E_result(i));

  	  			}

  	  		}


  	  	}

  	    //std::cout << "real E_result after change " << real_E_result << std::endl;



		//std::cout << "The eigenvalues of sta_EVP after elimination are:" << std::endl;
  	  	//std::cout << E_result<< std::endl;

		// Calculate the median 

  	  	/* // Matlab script

		
			[V, E] = polyeig(A_0, A_1, A_2);
			
			E_res_1 = ones(size(E));
			E_res_2 = ones(size(E));
			E_res_1(~isinf(E)) = 0;
			E(~E_res_1 == 0) = 0;
			E_res_2(imag(E) == 0 & real(E)~=0) = 0;

			

			median(real(E(find(~E_res_2))));
			prctile(real(E(find(~E_res_2))),10)
			prctile(real(E(find(~E_res_2))),90)

  	  	*/

  	  }
	  



	  // Solve the constraint equation for F from nullspace extraction.
	  // An LU decomposition is efficient for the minimally constrained case.
	  // Otherwise, use an SVD.
	  Matrix<double, 9, 1> normalized_fvector;
	  if (image_1_points.size() == 8) {
	    const auto lu_decomposition = constraint_matrix_D1.fullPivLu();
	    if (lu_decomposition.dimensionOfKernel() != 1) {
	      return false;
	    }
	    // kernel of matrix is also called NULL-SPACE. Columns of returned matrix will form a basis of the kernel
	    normalized_fvector = lu_decomposition.kernel();
	  } else {
	    JacobiSVD<Matrix<double, Eigen::Dynamic, 9> > cmatrix_svd(
	       constraint_matrix_D1, Eigen::ComputeFullV);
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

}
}