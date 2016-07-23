#ifndef VIDEO_IO_HPP
#define VIDEO_IO_HPP


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>

#include <string>
#include <iostream>

#include <Eigen/Dense>
#include <stdio.h>


namespace Open_SfM_METU {
namespace video {


    int readVideo(const char *  inPath, const char *  outPath);


}
}





#endif
