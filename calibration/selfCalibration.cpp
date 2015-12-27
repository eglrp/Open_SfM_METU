


#include "selfCalibration.hpp"



using Eigen::MatrixXd;

  

  // class constructor
  // imageler gelecek ve okuncak (hem opencv hem de openMVG ile okuyalim)
  // hangi featurelar ile calib yapacagimizi sececegiz

  // class functions 


  // init func olacak ve bunda featurelar cikartilacak 
  // bu feature lar ile matching yapilacak ve bu matchingler de cesitli olacak 
  // openMVG nin featue extraction ve matchingleri kulllanilabilir


selfCalibration::selfCalibration(int d, int m, int y)
{
  if(d>0 && d<31) day = d;
  if(m>0 && m<13) month = m;
  if(y>0) year =y;

  MatrixXd mat(2,2);
  mat(0,0) = 3;
  mat(1,0) = 2.5;
  mat(0,1) = -1;
  mat(1,1) = mat(1,0) + mat(0,1);
  std::cout << mat << std::endl;
}

selfCalibration::~selfCalibration(void){

	// delete all 
}

void selfCalibration::set(int d, int m, int y)
{
  if(d>0 && d<31) day = d;
  if(m>0 && m<13) month = m;
  if(y>0) year =y;
}

void selfCalibration::print()
{
  std::cout << day << "-" << month << "-" << year << std::endl;
}




