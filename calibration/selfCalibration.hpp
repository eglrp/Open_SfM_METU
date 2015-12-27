#ifndef SELF_CALIBRATION_HPP
#define SELF_CALIBRATION_HPP


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>

#include <string>
#include <iostream>

#include <Eigen/Dense>
#include <stdio.h>

#include "openMVG/image/image.hpp"
#include "Open_SfM_METU/feature/feature.hpp"



class selfCalibration {

public:  
  selfCalibration(int, int, int);
  ~selfCalibration();
  void set(int, int, int);
  void print();
  void findFocalLength(); // input of this functionis vector of images and the image type should be WHAT ??

private:
  int year;
  int month;
  int day;
};



#endif
