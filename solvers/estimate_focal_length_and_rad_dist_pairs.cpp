


#include <Eigen/Core>
#include <vector>
#include <iostream>
#include <random>

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>

#include "Open_SfM_METU/matching/feature_correspondence.hpp"
#include "Open_SfM_METU/solvers/estimator.hpp"
#include "Open_SfM_METU/solvers/pose_util.hpp"
#include "Open_SfM_METU/solvers/util_util.hpp"
#include "Open_SfM_METU/solvers/estimate_focal_length_and_rad_dist_pairs.hpp"

#include "Open_SfM_METU/solvers/pose/eight_point_fundamental_matrix_v2.hpp"
#include "Open_SfM_METU/solvers/estimate_focal_length_trace_const.hpp"



#include "Open_SfM_METU/solvers/random.hpp"
#include "Open_SfM_METU/calibration/camera_utils.hpp"

#include <glog/logging.h>

#include <math.h> 
#include <memory>
#include <algorithm>
#include <functional>


namespace Open_SfM_METU {
namespace solvers {

namespace {


// An estimator for computing the fundamental matrix from 8 feature
// correspondences. The feature correspondences should be in pixel coordinates.

class FundamentalMatrixEstimator_F
    : public Estimator<matching::FeatureCorrespondence, Eigen::Matrix3d> {


    public:
    	FundamentalMatrixEstimator_F() {}
    	~FundamentalMatrixEstimator_F() {}

    	// 8 correspondences are needed to determine an fundamental matrix.
		double SampleSize() const { return 8; }

		//std::vector<double> lambdaValues;

		// Estimates candidate fundamental matrices from correspondences.
		bool EstimateModel(const std::vector<matching::FeatureCorrespondence>& correspondences,
		                 std::vector<Eigen::Matrix3d>* fundamental_matrices) const {
			std::vector<Eigen::Vector2d> image1_points, image2_points;
			
			//std::cout << "correspondences size " << correspondences.size() << std::endl;

			for (int i = 0; i < 8; i++) {
			  image1_points.emplace_back(correspondences[i].feature1);
			  image2_points.emplace_back(correspondences[i].feature2);
			}


			Eigen::Matrix3d fmatrix;
			std::vector<double> lambdaValues;
			if (!NormalizedEightPointFundamentalMatrix_v2(image1_points, image2_points, &fmatrix)) {
			//if (!NormalizedEightPointFundamentalMatrixWithRadialDistortion(image1_points, image2_points, &fmatrix, lambdaValues)) {
			        
				
			  	return false; 
			}		

			//std::cout << "lambda values size " << std::endl;
			//std::cout << " size of lambda vector = > " <<lambdaValues.size() << std::endl;

			fundamental_matrices->emplace_back(fmatrix);
			return true;
		}

		// Estimates final fundamental matrices from correspondences.
		bool EstimateModelFinal(const std::vector<matching::FeatureCorrespondence>& correspondences,
		                 Eigen::Matrix3d* fundamental_matrix,
		                 std::vector<int>& bestInliers) const {			
			
			std::cout << "best inlier size " << bestInliers.size() << std::endl;
			std::cout << "corrs size " << correspondences.size() << std::endl;

			
			std::vector<Eigen::Vector2d> image1_points, image2_points;
			
			//std::cout << "correspondences size " << correspondences.size() << std::endl;

			for (int i = 0; i < bestInliers.size(); i++) {
			  //image1_points.emplace_back(correspondences[i].feature1);
			  //image2_points.emplace_back(correspondences[i].feature2);

			  //std::cout << "inlier index " << bestInliers[i] << std::endl;
			  image1_points.emplace_back(correspondences[bestInliers[i]].feature1);
			  image2_points.emplace_back(correspondences[bestInliers[i]].feature2);
			}

			Eigen::Matrix3d fmatrix;
			std::vector<double> lambdaValues;
			if (!NormalizedN_pointFundamentalMatrix_v2(image1_points, image2_points, fundamental_matrix)) {
			        				
			  	return false; 
			}	


			return true;

		}

				// Estimates candidate fundamental matrices from correspondences.
		bool EstimateModel(const std::vector<matching::FeatureCorrespondence>& correspondences,
		                 std::vector<Eigen::Matrix3d>* fundamental_matrices, std::vector<double>* l_values) const {

			return true;
		}

		bool EstimateModel(const std::vector<matching::FeatureCorrespondence>& correspondences,
		                 std::vector<Eigen::Matrix3d>* fundamental_matrices, std::vector<double>* l_values, int imageWidth, int numCorr) const {

			return true;
		}

		// The error for a correspondences given a model. This is the squared sampson
		// error.
		double Error(const matching::FeatureCorrespondence& correspondence,
		           const Eigen::Matrix3d& fundamental_matrix) const {
			return SquaredSampsonDistance(fundamental_matrix,
		                              correspondence.feature1,
		                              correspondence.feature2);
  		}

  	private:
  		DISALLOW_COPY_AND_ASSIGN(FundamentalMatrixEstimator_F);


};

} // namespace

int lowerLimit = 0;
int upperLimit = 0;

int sample(int dummy)
{
	//VLOG(3)<<"lower and upper limits are "<< lowerLimit << " and " << upperLimit;
	return (RandInt(lowerLimit, upperLimit));
}

Eigen::MatrixXi randomMatrixGenerator(int n_row, int n_col, int min_val, int max_val){

	lowerLimit = min_val;
	upperLimit = max_val;

	Eigen::MatrixXi m = Eigen::MatrixXi::Zero(n_row,n_col).unaryExpr(std::ptr_fun(sample));
  	//std::cout << m << std::endl;

  	return m;
}

bool polyEig(std::vector<Eigen::MatrixXd> _input_matrices_vec, int n_value, int p_value, Eigen::VectorXcd* E_result, Eigen::MatrixXcd* X_result){

	
	/*

	std::cout << "matrix_D3 is in polyeig" << std::endl;
	std::cout << _input_matrices_vec[0] << std::endl;


	std::cout << "matrix_D2 is in polyeig" << std::endl;
	std::cout << _input_matrices_vec[1] << std::endl;


	std::cout << "matrix_D1 is in polyeig" << std::endl;
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

	//Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> es_temp;
	Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> es_temp(A_mat,B_mat,true);
	
	//Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> es_temp;
	//Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXcd> es_temp(A_mat_c,B_mat_c);

	//es_temp.compute(A_mat,B_mat,true);


	//std::cout << "eigen values in for A and B system " << es_temp.eigenvalues() << std::endl;

	

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


bool EstimateFocalLengthAndRadialDistortionPairs(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    std::vector<matching::FeatureCorrespondence>& unnormalized_correspondences,
    Eigen::Matrix3d* fundamental_matrix,
    RansacSummary* ransac_summary,
    std::vector<double>* all_l_values,
    std::vector<double>* all_f_values, 
    std::vector<int>* FOV_interval) {
  
	LOG(INFO) << "Running Focal Length Rad Dist Pairs Estimation";

	InitRandomGenerator();

	int n_row = ransac_params.ite_num;
	int n_col = ransac_params.min_num;
	
	//VLOG(3) << "size of correspondences " << unnormalized_correspondences.size();

	std::cout << "unnormalized_correspondences sample X " << unnormalized_correspondences.at(0).feature1(0) << std::endl;
	std::cout << "unnormalized_correspondences sample Y " << unnormalized_correspondences.at(0).feature1(1) << std::endl;




	int min_val = 1;
	int max_val = unnormalized_correspondences.size() - 1;

	// Number of corrrespondences
	int n_corr = unnormalized_correspondences.size();

	Eigen::MatrixXi indexMat = randomMatrixGenerator(n_row, n_col, min_val, max_val);


for (int id_focal = 0; id_focal < FOV_interval->size(); id_focal++){


	VLOG(2) << "FOV is " << FOV_interval->at(id_focal) << std::endl;

	// akin double f_temp = camera::afov2focal((double)(FOV_interval->at(0)),ransac_params.image_width);
	double f_temp = camera::afov2focal((double)(FOV_interval->at(id_focal)),ransac_params.image_width);

	VLOG(2) << "focal length is " << f_temp << std::endl;


	double px = ransac_params.image_width  / 2.0f;
	double py = ransac_params.image_height / 2.0f;

	Eigen::Matrix3d K_int;
	K_int << f_temp, 0.0, px,
       		 0.0, f_temp, py,
       		 0.0, 0.0, 1;

    //std::cout << "K_int " << K_int << std::endl;

    //std::cout << "indexMat => " << indexMat << std::endl;

   	// Correspondence lar std::vector de bundan eigen matrise aktaralim.

    /*

	Eigen::MatrixXd corr_0(3, n_corr);
	Eigen::MatrixXd corr_1(3, n_corr);


	for(int i = 0; i < n_corr; ++i){

		//VLOG(2)<< "corr_point " << unnormalized_correspondences.at(i).feature1(0);
		
		corr_0(0,i) = unnormalized_correspondences.at(i).feature1(0);
		corr_0(1,i) = unnormalized_correspondences.at(i).feature1(1);
		corr_0(2,i) = 1;

		corr_1(0,i) = unnormalized_correspondences.at(i).feature2(0);
		corr_1(1,i) = unnormalized_correspondences.at(i).feature2(1);
		corr_1(2,i) = 1;
		
	}
	
	// Realization of unnormalized_correspondences
	//unnormalized_correspondences.at(i).feature1 = Feature(keypoint1.x(), keypoint1.y());
	//unnormalized_correspondences.at(i).feature2 = Feature(keypoint2.x(), keypoint2.y());

	//VLOG(2) << "corr 0 is (column 0) " << corr_0.col(0) << std::endl;

	// Normalize Image Coordinates


	Eigen::MatrixXd norm_corr_0(3, n_corr);
	Eigen::MatrixXd norm_corr_1(3, n_corr);

	norm_corr_0 = K_int.inverse() * corr_0;
	norm_corr_1 = K_int.inverse() * corr_1;

	*/

	// Write down the iteration part by utilizing the indexMat

	std::vector<double> lambdaValues;

	for(int ind_it = 0; ind_it < n_row; ++ind_it ){


		Eigen::MatrixXd sel_corr_0(3, n_col);
		Eigen::MatrixXd sel_corr_1(3, n_col);

		//std::cout << "row of index mat " << indexMat.row(ind_it) << std::endl;

		// Find the points to be used
		for(int p = 0; p < n_col; ++p){

			int index_point = indexMat(ind_it,p);

			

			sel_corr_0(0,p) = unnormalized_correspondences.at(index_point).feature1(0);
			sel_corr_0(1,p) = unnormalized_correspondences.at(index_point).feature1(1);
			sel_corr_0(2,p) = 1;

			sel_corr_1(0,p) = unnormalized_correspondences.at(index_point).feature2(0);
			sel_corr_1(1,p) = unnormalized_correspondences.at(index_point).feature2(1);
			sel_corr_1(2,p) = 1;	

			
			

		}

		Eigen::MatrixXd sel_norm_corr_0(3, n_col);
		Eigen::MatrixXd sel_norm_corr_1(3, n_col);


		sel_norm_corr_0 = K_int.inverse() * sel_corr_0;
		sel_norm_corr_1 = K_int.inverse() * sel_corr_1;

		/*

		std::cout << "sel_norm_corr_0 -> " << std::endl;
		std::cout <<  sel_norm_corr_0 << std::endl;


		std::cout << "sel_norm_corr_1 -> " << std::endl;
		std::cout <<  sel_norm_corr_1 << std::endl;

		*/

		// Constraint matrix D1
		Eigen::MatrixXd constraint_matrix_D1(n_col, 9);
		constraint_matrix_D1 = Eigen::MatrixXd::Zero(n_col, 9);


		for (int k = 0; k < n_col; ++k) {

			//std::cout << "frame 0" << sel_norm_corr_0.col(k) << std::endl;
			//std::cout << "frame 1" << sel_norm_corr_1.col(k) << std::endl;

			constraint_matrix_D1.block<1, 3>(k, 0) = sel_norm_corr_0.col(k);
			constraint_matrix_D1.block<1, 3>(k, 0) *= sel_norm_corr_1(0,k);
			constraint_matrix_D1.block<1, 3>(k, 3) = sel_norm_corr_0.col(k);
			constraint_matrix_D1.block<1, 3>(k, 3) *= sel_norm_corr_1(1,k);
			constraint_matrix_D1.block<1, 3>(k, 6) = sel_norm_corr_0.col(k);


		}

		//std::cout << "constraint_matrix_D1" << constraint_matrix_D1 << std::endl;



		// Constraint matrix D2
		Eigen::MatrixXd constraint_matrix_D2(n_col, 9);
		constraint_matrix_D2 = Eigen::MatrixXd::Zero(n_col, 9);


		Eigen::Vector2d zero_vec;
		zero_vec<<0,0; 
		for (int i = 0; i < n_col; i++) {

			// r_prime_square
			double r_square_image_point_2 = sel_norm_corr_1.block<2,1>(0,i).transpose() * sel_norm_corr_1.block<2,1>(0,i) ; //sel_norm_corr_1.col(i).transpose() * sel_norm_corr_1.col(i);
			// r_square
			double r_square_image_point_1 = sel_norm_corr_0.block<2,1>(0,i).transpose() * sel_norm_corr_0.block<2,1>(0,i) ; //sel_norm_corr_0.col(i).transpose() * sel_norm_corr_0.col(i);


			constraint_matrix_D2.block<1, 2>(i, 0) = zero_vec;
			constraint_matrix_D2(i, 2) = r_square_image_point_1 * sel_norm_corr_1(0,i);
			constraint_matrix_D2.block<1, 2>(i, 3) = zero_vec;
			constraint_matrix_D2(i, 5) = r_square_image_point_1 * sel_norm_corr_1(1,i);
			constraint_matrix_D2.block<1, 3>(i, 6) = sel_norm_corr_0.col(i);
			constraint_matrix_D2.block<1, 3>(i, 6) *= r_square_image_point_2;
			constraint_matrix_D2(i, 8) += r_square_image_point_1;
		}

		//std::cout << "constraint_matrix_D2" << constraint_matrix_D2 << std::endl;



		// Constraint matrix D3
		Eigen::MatrixXd constraint_matrix_D3(n_col, 9);
		constraint_matrix_D3 = Eigen::MatrixXd::Zero(n_col, 9);



		Eigen::VectorXd zero_vec_2(8);
		zero_vec_2<<0,0,0,0,0,0,0,0; 

		for (int i = 0; i < n_col; i++) {

			// r_prime_square
			double r_square_image_point_2 = sel_norm_corr_1.block<2,1>(0,i).transpose() * sel_norm_corr_1.block<2,1>(0,i) ;
			// r_square
			double r_square_image_point_1 = sel_norm_corr_0.block<2,1>(0,i).transpose() * sel_norm_corr_0.block<2,1>(0,i) ;

			constraint_matrix_D3.block<1, 8>(i, 0) = zero_vec_2;
			constraint_matrix_D3(i, 8) = r_square_image_point_2 * r_square_image_point_1;


		}

		//std::cout << "constraint_matrix_D3" << constraint_matrix_D3 << std::endl;


		Eigen::MatrixXd constraint_matrix_D1_result;
		Eigen::MatrixXd constraint_matrix_D2_result;
		Eigen::MatrixXd constraint_matrix_D3_result;

		constraint_matrix_D1_result = constraint_matrix_D1.transpose() * constraint_matrix_D1;
		constraint_matrix_D2_result = constraint_matrix_D1.transpose() * constraint_matrix_D2;
		constraint_matrix_D3_result = constraint_matrix_D1.transpose() * constraint_matrix_D3;

		//std::cout << "D1" << constraint_matrix_D1_result << std::endl;
		//std::cout << "D2" << constraint_matrix_D2_result << std::endl;
		//std::cout << "D3" << constraint_matrix_D3_result << std::endl;


		std::vector<Eigen::MatrixXd> input_matrices_vec;
		input_matrices_vec.push_back(constraint_matrix_D3_result);
		input_matrices_vec.push_back(constraint_matrix_D2_result);
		input_matrices_vec.push_back(constraint_matrix_D1_result);


		Eigen::VectorXcd E_result;
		Eigen::MatrixXcd X_result;

		int n_value =  constraint_matrix_D1_result.rows();
		int p_value =  input_matrices_vec.size() - 1;


		if(polyEig(input_matrices_vec, n_value, p_value, &E_result, &X_result)){

			//VLOG(2) << "The eigenvalues of sta_EVP are:" << E_result;


			Eigen::VectorXd real_E_result(9);
			real_E_result << 0,0,0,0,0,0,0,0,0;

			//std::cout << "E_result " << E_result << std::endl;

			for(int i = 0; i < 9 ; ++i){

				if(E_result(i).imag() == 0){

					if(!std::isnan(E_result(i).real()) && !std::isinf(E_result(i).real())){

						if((std::abs(E_result(i).real())) > 0.1 ){

							real_E_result(i) = E_result(i).real();

							lambdaValues.push_back(1.0f / real_E_result(i));
							// gelen eigen valularin sanirim tersi kullanilacak
							// bir de polyeig sonuclarinda abs elemesi vardi bunda da ters eigen value ya dikkat edelim. 
							// buna ek olarak polyeig icinde matlabda yaptigimizdaki ayni normalized koordinatlari kullaniuor muyuz ??
							// polyeig e matrisleri verirken soyle ypalim. paperdaki fonksiyon dogru ve variable lar da dogru. 
							// matlabin polyeig fonksiyonundaki variable sirasi yanlis. 

						}



					}

				}


			}

		}



	}

	//lambdaValues ile ne yapalim. Butun iterationlardan gelen lambda value larinin hepsi burada bir focal length degeri icin. 

	std::sort(lambdaValues.begin(), lambdaValues.end());

	double median = *(lambdaValues.begin()+lambdaValues.size()/2); 

	/*

	for (int id_l = 0; id_l < lambdaValues.size(); ++id_l){

		std::cout << lambdaValues[id_l] << ";" ;

	}

	*/

	
	//VLOG(2) << "Size of lambdaValues " << lambdaValues.size();
	//VLOG(2) << "Median value of calculated eigen values is " << median;


	all_l_values->push_back(median);

	all_f_values->push_back(f_temp);



}



	// Unnormalized Coordinates before undistortion 

	

	std::cout << "frame 0 is " << std::endl;


	for(int ind_k = 0; ind_k < unnormalized_correspondences.size(); ind_k++){

		std::cout << unnormalized_correspondences.at(ind_k).feature1(0) << "," << unnormalized_correspondences.at(ind_k).feature1(1) << ";" ;

	}

	std::cout << "frame 1 is " << std::endl;

	for(int ind_k = 0; ind_k < unnormalized_correspondences.size(); ind_k++){

		std::cout << unnormalized_correspondences.at(ind_k).feature2(0) << "," << unnormalized_correspondences.at(ind_k).feature2(1) << ";" ;

	}

	



// Estimate the Fundamental Matrix from undistorted, matched image coordinates

	// undistort the image correspondences 

	// std::vector<matching::FeatureCorrespondence>& unnormalized_correspondences 

	
	Eigen::Matrix3d K_mat;
	double f_t = all_f_values->at(1);
	double px_t = ransac_params.image_width  / 2.0f;
	double py_t = ransac_params.image_height / 2.0f;
	K_mat << f_t, 0.0, px_t,
	     0.0, f_t, py_t,
	     0.0, 0.0, 1;

	double lambda_f_est = all_l_values->at(1);

	VLOG(2) << "f value = " << f_t;
	VLOG(2) << "l value = " << lambda_f_est;

	std::vector<matching::FeatureCorrespondence> unnormalized_correspondences_undistorted;

	for(int corr_ind = 0; corr_ind < unnormalized_correspondences.size(); corr_ind++){

		/*

		std::cout << "old value Frame 0" << std::endl;
		std::cout << unnormalized_correspondences.at(corr_ind).feature1(0) << " ___ " << 
				unnormalized_correspondences.at(corr_ind).feature1(1) << std::endl;

		*/


		//////////////
		// Frame 0
		/////////////

		Eigen::Vector3d temp_IN_3Dpoint_1;
    	Eigen::Vector3d temp_OUT_3Dpoint_1;
    	Eigen::Vector2d temp_IN_2D_point_1;
    	Eigen::Vector2d temp_OUT_2D_point_1;

	
		temp_IN_3Dpoint_1 << unnormalized_correspondences.at(corr_ind).feature1(0),
							unnormalized_correspondences.at(corr_ind).feature1(1), 1;


		temp_OUT_3Dpoint_1 =  K_mat.inverse() * temp_IN_3Dpoint_1;

		temp_IN_2D_point_1(0) = temp_OUT_3Dpoint_1(0);
		temp_IN_2D_point_1(1) = temp_OUT_3Dpoint_1(1);

		if(camera::undistortImagePoint(temp_IN_2D_point_1, temp_OUT_2D_point_1, lambda_f_est)){
			//VLOG(3) << "point from frame 0 is undistorted";
		}

		Eigen::Vector3d temp_3D_IN_res_1;
		Eigen::Vector3d temp_3D_OUT_res_1;
		temp_3D_IN_res_1 << temp_OUT_2D_point_1(0), temp_OUT_2D_point_1(1), 1;

		temp_3D_OUT_res_1 = K_mat * temp_3D_IN_res_1;		

		



		unnormalized_correspondences.at(corr_ind).feature1(0) = temp_3D_OUT_res_1(0) - px_t;
		unnormalized_correspondences.at(corr_ind).feature1(1) = temp_3D_OUT_res_1(1) - py_t;




		//////////////
		// Frame 1
		/////////////

		Eigen::Vector3d temp_IN_3Dpoint_2;
    	Eigen::Vector3d temp_OUT_3Dpoint_2;
    	Eigen::Vector2d temp_IN_2D_point_2;
    	Eigen::Vector2d temp_OUT_2D_point_2;


				
    	temp_IN_3Dpoint_2 << unnormalized_correspondences.at(corr_ind).feature2(0),
							unnormalized_correspondences.at(corr_ind).feature2(1), 1;

	
		temp_OUT_3Dpoint_2 = K_mat.inverse() * temp_IN_3Dpoint_2;

		temp_IN_2D_point_2(0) = temp_OUT_3Dpoint_2(0);
		temp_IN_2D_point_2(1) = temp_OUT_3Dpoint_2(1);


		if(camera::undistortImagePoint(temp_IN_2D_point_2, temp_OUT_2D_point_2, lambda_f_est)){
			//VLOG(3) << "point from frame 1 is undistorted";
		}

		Eigen::Vector3d temp_3D_IN_res_2;
		Eigen::Vector3d temp_3D_OUT_res_2;
		temp_3D_IN_res_2 << temp_OUT_2D_point_2(0), temp_OUT_2D_point_2(1), 1;

		temp_3D_OUT_res_2 = K_mat * temp_3D_IN_res_2;	

		

		unnormalized_correspondences.at(corr_ind).feature2(0) = temp_3D_OUT_res_2(0) - px_t;
		unnormalized_correspondences.at(corr_ind).feature2(1) = temp_3D_OUT_res_2(1) - py_t;

		// unnormalized_corr icine nasil ekleyecegiz pointlerin yeni halini. bir de bu noktalar 
		// integer a cast olmali coordinate olabilmek icin. 

		// akin unnormalized_correspondences.at(corr_ind) = temp_Feat_Corr;


				

		if(temp_3D_OUT_res_1(0) >= 0 && temp_3D_OUT_res_1(0) <= ransac_params.image_width && 
				temp_3D_OUT_res_1(1) >= 0 && temp_3D_OUT_res_1(1) <= ransac_params.image_height &&
					temp_3D_OUT_res_2(0) >= 0 && temp_3D_OUT_res_2(0) <= ransac_params.image_width && 
						temp_3D_OUT_res_2(1) >= 0 && temp_3D_OUT_res_2(1) <= ransac_params.image_height ) {

			matching::FeatureCorrespondence temp_Feat_Corr;

			matching::Feature feature1; 
			//feature1 << temp_OUT_2D_point_1(0), temp_OUT_2D_point_1(1);
			feature1 << (temp_3D_OUT_res_1(0) - px_t), (temp_3D_OUT_res_1(1) - py_t);
			temp_Feat_Corr.feature1 = feature1;

			matching::Feature feature2; 
			feature2 << (temp_3D_OUT_res_2(0) - px_t), (temp_3D_OUT_res_2(1) - py_t);
			//feature2 << temp_OUT_2D_point_2(0), temp_OUT_2D_point_2(1);
			temp_Feat_Corr.feature2 = feature2;	

			unnormalized_correspondences_undistorted.push_back(temp_Feat_Corr);

		}

		
		


		/*

		std::cout << "new value Frame 0 " << std::endl;
		std::cout << unnormalized_correspondences.at(corr_ind).feature1(0) << " ___ " << 
					unnormalized_correspondences.at(corr_ind).feature1(1) << std::endl;

		*/


	}

	std::cout << "size of correspondence is unnormalized_correspondences =  " << unnormalized_correspondences.size() << std::endl;
	std::cout << "size of correspondence is unnormalized_correspondences_undistorted = " << unnormalized_correspondences_undistorted.size() << std::endl;


	/*

	std::cout << "frame 0 is " << std::endl;


	for(int ind_k = 0; ind_k < unnormalized_correspondences_undistorted.size(); ind_k++){

		std::cout << unnormalized_correspondences_undistorted.at(ind_k).feature1(0) << "," << unnormalized_correspondences_undistorted.at(ind_k).feature1(1) << ";" <<std::endl;

	}

	std::cout << "frame 1 is " << std::endl;

	for(int ind_k = 0; ind_k < unnormalized_correspondences_undistorted.size(); ind_k++){

		std::cout << unnormalized_correspondences_undistorted.at(ind_k).feature2(0) << "," << unnormalized_correspondences_undistorted.at(ind_k).feature2(1) << ";" << std::endl;

	}

	*/

	// F estimation  

    RansacParameters options_F;
    options_F.use_mle = false;
    options_F.error_thresh = 1; 
    options_F.min_inlier_ratio = 0.99;
    options_F.min_iterations = 300;
    options_F.failure_probability = 0.01;
    options_F.image_height = ransac_params.image_height;
    options_F.image_width = ransac_params.image_width;
    options_F.num_pointCorr = 200;
    options_F.ite_num = 1000;
    options_F.min_num = 10;
	


	Eigen::Matrix3d F_matrix;
    RansacSummary ransac_summary_F;
    std::vector<double> result_all_eig_values;


    FundamentalMatrixEstimator_F fundamental_matrix_estimator;
	std::unique_ptr<SampleConsensusEstimator<FundamentalMatrixEstimator_F> >
	  ransac = CreateAndInitializeRansacVariant(RansacType::RANSAC,
	                                            options_F,
	                                            fundamental_matrix_estimator);

	bool result = ransac->Estimate(unnormalized_correspondences_undistorted,
	                      &F_matrix,
	                      &ransac_summary_F,
	                      &result_all_eig_values);


	std::cout << "F matrix is " << F_matrix << std::endl;

	// Estimate the focal length from Fundamental Matrix using both Sturm and Our Method

	double f_length;
	if(EstimateFocalLengthTraceConst(F_matrix, f_length)){

		std::cout << "focal length is estimated" << std::endl;
	}
	



	return true; 

}




}
}