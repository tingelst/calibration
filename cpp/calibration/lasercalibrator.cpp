/*
 * lasercalibrator.cpp
 *
 *  Created on: Nov 21, 2012
 *      Author: lars
 */

// Standard includes
#include <iostream>
#include <vector>
// OpenCV includes
#include <opencv2/core/core.hpp>
#include "opencv2/calib3d/calib3d.hpp"
// Local includes
#include "lasercalibrator.h"

using namespace cv;
using namespace std;

LaserCalibrator::LaserCalibrator() {
	//TODO Not implemented yet

}

LaserCalibrator::~LaserCalibrator() {
  // TODO Auto-generated destructor stub
}

bool LaserCalibrator::get_chessboard_pose(vector<Point3f>& object_points,
		vector<Point2f>& image_points, Mat& rvec, Mat& tvec) {
	solvePnP(Mat(object_points), Mat(image_points), this->get_camera_matrix(), this->get_dist_coeffs(), rvec, tvec, false);

}


