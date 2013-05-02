/*
 * camera_capture.cpp
 *
 *  Created on: Nov 21, 2012
 *      Author: lars
 */

#include "opencv2/core/core.hpp"
#include "camera_capture.h"

using namespace cv;
using namespace std;

CameraCapture::CameraCapture() {
  cap.open(0);
}

CameraCapture::~CameraCapture() {
  // TODO Auto-generated destructor stub
}

Mat CameraCapture::read_(Mat& image) {
  cap >> image;
  return image;
}


