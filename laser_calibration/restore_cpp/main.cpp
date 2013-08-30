/*
 * main.cpp
 *
 *  Created on: May 29, 2013
 *      Author: lars
 */

#include <iostream>
#include <vector>

#include <chrono>


#include <opencv2/opencv.hpp>

#include "scanner.h"

#include <time.h>

using namespace std;
using namespace cv;
using namespace laser_scanning_library;

int main(int argc, char** argv) {

  Mat cm, dc, rmat, tvec, lpoint, lplane;

  // Read in matrices from file
  FileStorage cm_fs("/home/lars/devel/calibration/cal_data/cm.xml", FileStorage::READ);
  FileStorage dc_fs("/home/lars/devel/calibration/cal_data/dc.xml", FileStorage::READ);
  FileStorage rmat_fs("/home/lars/devel/calibration/cal_data/c2w_rmat.xml", FileStorage::READ);
  FileStorage tvec_fs("/home/lars/devel/calibration/cal_data/c2w_tvec.xml", FileStorage::READ);
  FileStorage lplane_fs("/home/lars/devel/calibration/cal_data/lplane.xml", FileStorage::READ);
  FileStorage lpoint_fs("/home/lars/devel/calibration/cal_data/lpoint.xml", FileStorage::READ);

  cm_fs["cm"] >> cm;
  dc_fs["dc"] >> dc;
  rmat_fs["rmat"] >> rmat;
  tvec_fs["tvec"] >> tvec;
  lplane_fs["lplane"] >> lplane;
  lpoint_fs["lpoint"] >> lpoint;

  // test laser line
  FileStorage ll_fs("/home/lars/devel/calibration/cal_data/laserline2d.xml", FileStorage::READ);

  Mat laserline(2048, 2, CV_64F);
  ll_fs["laserline2d"] >> laserline;
  laserline = laserline.t();

//  cout << laserline.cols << endl;
//  cout << laserline.rows << endl;

  Mat laserline2(2048, 2, CV_64F);
  vector<double> range;
  for (double i = 0; i < 2048; i++)
    range.push_back(i);
  Mat xrange(range);

  xrange.col(0).copyTo(laserline2.col(0));
  laserline2.col(1) = laserline.col(1) / 64.0;
  laserline2 = laserline2.reshape(2, 0);
//  cout << laserline2.at<Point2f>(0) << endl;
  Scanner scanner(cm, dc, rmat, tvec, lplane, lpoint);
  auto t1 = std::chrono::high_resolution_clock::now();
  auto n = 100.0;
  for (int i=0; i<n; i++)
    vector<double> p3ds = scanner.get3DCoords(laserline2);
  auto t2 = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() / 1000.0;
  auto fps = n / duration ;
  cout << "1 samples took "
                << duration
                << " milliseconds"
                << endl;
  cout << "fps: " << fps << endl;
  return 0;
}

