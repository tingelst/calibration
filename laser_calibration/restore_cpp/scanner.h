/*
 * scanner.h
 *
 *  Created on: May 29, 2013
 *      Author: lars
 */

#ifndef SCANNER_H_
#define SCANNER_H_

#include <opencv2/opencv.hpp>

namespace laser_scanning_library {

using namespace cv;

class Scanner {
 public:
  Scanner(Mat cm, Mat dc, Mat rmat, Mat tvec, Mat lplane, Mat lpoint)
      : cm_(cm),
        dc_(dc),
        rmat_(rmat),
        inv_rmat_(rmat.t()),
        tvec_(tvec),
        lplane_(lplane),
        lpoint_(lpoint),
        p3ds_(2048 * 3) {
  }
  virtual ~Scanner();

  vector<double> get3DCoords(Mat laserline2d);
  vector<double> linePlaneIntersect(Point2d point);

 private:

  Mat cm_;
  Mat dc_;
  Mat rmat_;
  Mat inv_rmat_;
  Mat tvec_;
  Mat lplane_;
  Mat lpoint_;
  vector<double> p3ds_;
};

} /* namespace laser_scanning_library */
#endif /* SCANNER_H_ */
