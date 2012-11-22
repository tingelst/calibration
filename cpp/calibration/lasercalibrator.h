/*
 * lasercalibrator.h
 *
 *  Created on: Nov 21, 2012
 *      Author: lars
 */

#ifndef LASERCALIBRATOR_H_
#define LASERCALIBRATOR_H_

#include "vector"
#include "opencv2/core/core.hpp"
#include "calibrator.h"

using namespace cv;
using namespace std;

class LaserCalibrator: public Calibrator {
public:
	LaserCalibrator();
	virtual ~LaserCalibrator();
	bool get_chessboard_pose(vector<Point3f> &object_points,
			vector<Point2f> &image_points, Mat& rvec, Mat& tvec);
};

#endif /* LASERCALIBRATOR_H_ */
