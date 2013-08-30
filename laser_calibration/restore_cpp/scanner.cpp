/*
 * scanner.cpp
 *
 *  Created on: May 29, 2013
 *      Author: lars
 */

#include "scanner.h"

namespace laser_scanning_library {

Scanner::~Scanner() {
  // TODO Auto-generated destructor stub
}

vector<double> Scanner::get3DCoords(Mat laserline2d) {
  p3ds_.clear();
  Mat udp(laserline2d);
  undistortPoints(laserline2d, udp, cm_, dc_);
  vector<double> p3d(3);
  for (int i = 0; i < laserline2d.rows; i++) {
    for (int j = 0; j < 3; j++) {
      p3d = linePlaneIntersect(udp.at<Point2f>(i));
//      std::cout << p3d.at(j) << " ";
      p3ds_.push_back(p3d.at(j));
    }
//    std::cout << std::endl;
  }
  return p3ds_;
}

vector<double> Scanner::linePlaneIntersect(Point2d undistorted_point) {

  Matx31f pc(undistorted_point.x, undistorted_point.y, 1);
  Matx33f rmat(rmat_);
  Matx33f inv_rmat(inv_rmat_);
  Matx31f tvec(tvec_);

  Matx31f pw = inv_rmat * (pc - tvec);
  Matx31f l0 = inv_rmat * (-tvec);
  Matx31f l = pw - l0;
  double ll = norm(l);
  if (ll != 0)
    (1 / ll) * l;

  Matx31f n(lplane_);
  Matx31f p0(lpoint_);

  double d1 = (p0 - l0).dot(n);
  double d2 = l.dot(n);

  Matx31f p3d = (d1 / d2) * l + l0;

  vector<double> p3dvec;
  p3dvec.push_back(p3d(0));
  p3dvec.push_back(p3d(1));
  p3dvec.push_back(p3d(2));

  // world coordinates
  return p3dvec;
}

} /* namespace laser_scanning_library */
