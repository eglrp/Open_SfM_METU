#ifndef OPEN_SFM_METU_SOLVERS_ESTIMATE_FOCAL_LENGTH_TRACE_CONST_HPP
#define OPEN_SFM_METU_SOLVERS_ESTIMATE_FOCAL_LENGTH_TRACE_CONST_HPP


#include <Eigen/Core>
#include <vector>

#include <iostream>

#include "Open_SfM_METU/math/find_polynomial_roots_companion_matrix.hpp"



namespace Open_SfM_METU {
namespace solvers {


// Estimates the focal length and radial distortion coefficient pairs by utilizing 
// "Simultaneous linear estimation of multiple view geometry and lens distortion"



inline bool EstimateFocalLengthTraceConst(Eigen::Matrix3d& F_M, double& focal_length){

	std::cout << "Fundamental matrix is (trace const func) " << F_M << std::endl;

	double f11, f12, f13, f21, f22, f23, f31, f32, f33;



	f11 = F_M(0,0);
	f12 = F_M(0,1);
	f13 = F_M(0,2);

	f21 = F_M(1,0);
	f22 = F_M(1,1);
	f23 = F_M(1,2);

	f31 = F_M(2,0);
	f32 = F_M(2,1);
	f33 = F_M(2,2);

	// 1st element of vector

	double a11, a12, a13, a14;

	a11 = (2*f11*f11*f11 + 2*f11*f12*f12 + 2*f11*f21*f21 + 2*f12*f22*f21 - 
			(f11*f11*f11) - (f12*f12*f11) - (f21*f21*f11) - (f22*f22*f11));
	a12 = (2*f11*f13*f13 + 2*f11*f31*f31 + 2*f12*f32*f31 - (f13*f13*f11) - 
			(f23*f23*f11) - (f31*f31*f11) - (f32*f32*f11));
	a13 = (2*f13*f23*f21);
	a14 = (2*f13*f33*f31 - (f33*f33*f11));


	// 2nd element of vector

	double a21, a22, a23, a24;

	a21 = (2*f11*f11*f12 + 2*f12*f12*f12 + 2*f11*f21*f22 + 2*f12*f22*f22 - 
    	(f11*f11*f12) - (f12*f12*f12) - (f21*f21*f12) - (f22*f22*f12));
	a22 = (2*f13*f13*f12 + 2*f11*f31*f32 + 2*f12*f32*f32 - 
    	(f13*f13*f12) - (f23*f23*f12) - (f31*f31*f12) - (f32*f32*f12));
	a23 = (2*f13*f23*f22);
	a24 = (2*f13*f33*f32 - (f33*f33*f12));

	// 3rd element of vector

	double a31, a32, a33, a34;

	a31 = (2*f11*f11*f13 + 2*f12*f12*f13 + 2*f11*f21*f23 + 2*f12*f22*f23 - 
    	(f11*f11*f13) - (f12*f12*f13) - (f21*f21*f13) - (f22*f22*f13));
	a32 = (2*f13*f13*f13 + 2*f11*f31*f33 + 2*f12*f32*f33 - 
    	(f13*f13*f13) - (f23*f23*f13) - (f31*f31*f13) - (f32*f32*f13));
	a33 = (2*f13*f23*f23);
	a34 = (f13*f33*f33 - (f33*f33*f13));

	// 4th element of vector

	double a41, a42, a43;

	a41 = (2*f21*f11*f11 + 2*f22*f12*f11 + 2*f21*f21*f21 + 2*f22*f22*f21 - 
    	(f11*f11*f21) - (f12*f12*f21) - (f21*f21*f21) - (f22*f22*f21));
	a42 = (2*f23*f13*f11 + 2*f23*f23*f21 + 2*f21*f31*f31 + 2*f22*f32*f31 - 
    	(f13*f13*f21) - (f23*f23*f21) - (f31*f31*f21) - (f32*f32*f21));
	a43 = (2*f23*f33*f31 - (f33*f33*f21));

	// 5th element of vector

	double a51, a52, a53;

	a51 = (2*f21*f11*f12 + 2*f22*f12*f12 + 2*f21*f21*f22 + 2*f22*f22*f22 - 
    	(f11*f11*f22) - (f12*f12*f22) - (f21*f21*f22) - (f22*f22*f22));
	a52 = (2*f23*f13*f12 + 2*f23*f23*f22 + 2*f21*f31*f32 + 2*f22*f32*f32 - 
    	(f13*f13*f22) - (f23*f23*f22) - (f31*f31*f22) - (f32*f32*f22));
	a53 = (2*f23*f33*f32 - (f33*f33*f22));

	// 6th element of vector

	double a61, a62, a63;

	a61 = (2*f21*f11*f13 + 2*f22*f12*f13 + 2*f21*f21*f23 + 2*f22*f22*f23 - 
    	(f11*f11*f23) - (f12*f12*f23) - (f21*f21*f23) - (f22*f22*f23));
	a62 = (2*f23*f13*f13 + 2*f23*f23*f23 + 2*f21*f31*f33 + 2*f22*f32*f33 - 
    	(f13*f13*f23) - (f23*f23*f23) - (f31*f31*f23) - (f32*f32*f23));
	a63 = (2*f23*f33*f33 - (f33*f33*f23));

	// 7th element of vector

	double a71, a72, a73;

	a71 = (2*f31*f11*f11 + 2*f32*f12*f11 + 2*f31*f21*f21 + 2*f32*f22*f21 - 
    	(f11*f11*f31) - (f12*f12*f31) - (f21*f21*f31) - (f22*f22*f31));
	a72 = (2*f33*f13*f11 + 2*f33*f23*f21 + 2*f31*f31*f31 + 2*f32*f32*f31 - 
    	(f13*f13*f31) - (f23*f23*f31) - (f31*f31*f31) - (f32*f32*f31));
	a73 = (2*f33*f33*f31 - f33*f33*f31);


	// 8th element of vector

	double a81, a82, a83;

	a81 = (2*f31*f11*f12 + 2*f32*f12*f12 + 2*f31*f21*f22 + 2*f32*f22*f22 - 
    	(f11*f11*f32) - (f12*f12*f32) - (f21*f21*f32) - (f22*f22*f32));
	a82 = (2*f33*f13*f12 + 2*f33*f23*f22 + 2*f31*f31*f32 + 2*f32*f32*f32 - 
    	(f13*f13*f32) - (f23*f23*f32) - (f31*f31*f32) - (f32*f32*f32));
	a83 = (2*f33*f33*f32 - (f33*f33*f32));


	// 9th element of vector

	double a91, a92, a93;

	a91 = (2*f31*f11*f13 + 2*f32*f12*f13 + 2*f31*f21*f23 + 2*f32*f22*f23 - 
    	(f11*f11*f33) - (f12*f12*f33) - (f21*f21*f33) - (f22*f22*f33));
	a92 = (2*f33*f13*f13 + 2*f33*f23*f23 + 2*f31*f31*f33 + 2*f32*f32*f33 - 
    	(f13*f13*f33) - (f23*f23*f33) - (f31*f31*f33) - (f32*f32*f33));
	a93 = (2*f33*f33 - (f33*f33));


	///////////////////////////////////
	// Error Function Decleration /////
	///////////////////////////////////


	double k1, k2, k3, k4, k5, k6, k7, k8, k9, k10, k11;

	k1 = (a11*a11 + a21*a21 + a41*a41 + a51*a51);
	k2 = (a11*a12 + a12*a11 + a21*a22 + a22*a21 + a31*a31 + a41*a42 + 
    		a42*a41 + a51*a52 + a52*a51 + a61*a61 + a71*a71 + a81*a81);
	k3 = (a11*a13 + a13*a11 + a21*a23 + a23*a21);
	k4 = (a11*a14 + a12*a12 + a14*a11 + a21*a24 + a22*a22 + a24*a21 + 
		    a31*a32 + a32*a31 + a41*a43 + a42*a42 + a43*a41 + a51*a53 + 
		    a52*a52 + a53*a51 + a61*a62 + a62*a61 + a71*a72 + a72*a71 + 
		    a81*a82 + a82*a81 + a91*a91);
	k5 = (a12*a13 + a13*a12 + a22*a23 + a23*a22 + a31*a33 + a33*a31);
	k6 = (a12*a14 + a13*a13 + a14*a12 + a14*a13 + a22*a24 + a23*a23 + 
	    a24*a22 + a31*a34 + a32*a32 + a34*a31 + a42*a43 + a43*a42 + 
	    a52*a53 + a53*a52 + a61*a63 + a62*a62 + a63*a61 + a71*a73 +  
	    a72*a72 + a73*a71 + a81*a83 + a82*a82 + a83*a81 + a91*a92 + a92*a91);
	k7 = (a13*a14 + a23*a24 + a24*a23 + a32*a33 + a33*a32);
	k8 = (a14*a14 + a24*a24 + a32*a34 + a33*a33 + a34*a32 + a43*a43 + 
	    a53*a53 + a62*a63 + a63*a62 + a72*a73 + a73*a72 + a82*a83 + 
	    a83*a82 + a91*a93 + a92*a92 + a91*a93);
	k9 = (a33*a34 + a34*a33);
	k10 = (a34*a34 + a63*a63 + a73*a73 + a83*a83 + a92*a93 + a93*a92);
	k11 = (a93*a93);



	Eigen::VectorXd polynomial_in(11);
	polynomial_in << (12*k1), 0, (10*k2), (9*k3), (8*k4), (7*k5), (6*k6), (5*k7), (4*k8), (3*k9), (2*k10);  

	Eigen::VectorXd real;
	Eigen::VectorXd imaginary;



	// Burada sicti
	if(math::FindPolynomialRootsCompanionMatrix(polynomial_in,
                                        &real,
                                        &imaginary)){

		std::cout << "roots are (real)" << std::endl;
		std::cout <<  real << std::endl;

		std::cout << "roots are (imaginery)" << std::endl;
		std::cout <<  imaginary << std::endl;


	}
	

	return true;
}

}
}





#endif // OPEN_SFM_METU_SOLVERS_ESTIMATE_FOCAL_LENGTH_TRACE_CONST_HPP