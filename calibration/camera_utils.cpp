



#include "Open_SfM_METU/calibration/camera_utils.hpp"
#include <math.h>

namespace Open_SfM_METU {
namespace camera {

#define _USE_MATH_DEFINES

// Normalized image points are used. 
bool undistortImagePoint(Eigen::Vector2d& norm_inputPoint2d, Eigen::Vector2d& norm_outputPoint2d, double lambda){

	//std::cout << "input norm dist point " << norm_inputPoint2d << std::endl;

	double r_sq = (norm_inputPoint2d(0)*norm_inputPoint2d(0)) + (norm_inputPoint2d(1)*norm_inputPoint2d(1));

	norm_outputPoint2d(0) = norm_inputPoint2d(0) / (1 + ( lambda * (r_sq) ));
	norm_outputPoint2d(1) = norm_inputPoint2d(1) / (1 + ( lambda * (r_sq) ));

	//std::cout << "output norm dist point " << norm_outputPoint2d << std::endl;

	return true;
}

double afov2focal(double AFOV, double h){

	return ((h / 2.0) / (tan((AFOV / 2.0) * M_PI / 180)));
}

double focal2afov(double focal, double h){

	return (2.0 * atan(h / (2.0 * focal)) * 180 / M_PI);

}

bool undistortImage(image::Image<unsigned char>& inputImage, image::Image<unsigned char>& outputImage, Eigen::Matrix3d& K_mat, double lambda){

	std::cout << "lambda value is " << lambda << std::endl;

	for(int im_row = 0; im_row < inputImage.Height(); ++im_row){
        for(int im_col = 0; im_col < inputImage.Width(); ++im_col){

        	outputImage(im_row, im_col) = 0;
        	
        }

    }

	for(int im_row = 0; im_row < inputImage.Height(); ++im_row){
        for(int im_col = 0; im_col < inputImage.Width(); ++im_col){

        	Eigen::Vector3d temp_IN_3Dpoint;
        	Eigen::Vector3d temp_OUT_3Dpoint;
        	Eigen::Vector2d temp_IN_2D_point;
        	Eigen::Vector2d temp_OUT_2D_point;
			
			temp_IN_3Dpoint << im_col, im_row, 1;

			temp_OUT_3Dpoint = K_mat.inverse() * temp_IN_3Dpoint;

			temp_IN_2D_point(0) = temp_OUT_3Dpoint(0);
			temp_IN_2D_point(1) = temp_OUT_3Dpoint(1);

			if(undistortImagePoint(temp_IN_2D_point, temp_OUT_2D_point, lambda)){
				VLOG(3) << "point is undistorted";
			}

			Eigen::Vector3d temp_3D_IN_res;
			Eigen::Vector3d temp_3D_OUT_res;
			temp_3D_IN_res << temp_OUT_2D_point(0), temp_OUT_2D_point(1), 1;

			temp_3D_OUT_res = K_mat * temp_3D_IN_res;

			if((temp_3D_OUT_res(1) >= 0) && (temp_3D_OUT_res(1) <= inputImage.Height()) && (temp_3D_OUT_res(0) <= inputImage.Width()) && (temp_3D_OUT_res(0) >= 0)){

				//outputImage((int)temp_3D_OUT_res(0),(int)temp_3D_OUT_res(1)) = inputImage(im_row, im_col);
				outputImage((int)temp_3D_OUT_res(1),(int)temp_3D_OUT_res(0)) = inputImage(im_row, im_col);

			}


        }
    }

	return true;
}


} // camera
} // Open_SfM_METU