/*
 * camera_capture.h
 *
 *  Created on: Nov 21, 2012
 *      Author: lars
 */

#ifndef CAMERA_CAPTURE_H_
#define CAMERA_CAPTURE_H_

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

class CameraCapture {
 public:
  CameraCapture();
  virtual ~CameraCapture();
  Mat read_(Mat& image);

 private:
  VideoCapture cap;
};

#endif /* CAMERA_CAPTURE_H_ */
