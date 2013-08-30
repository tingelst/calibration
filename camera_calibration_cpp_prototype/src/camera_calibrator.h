/*
 * cameracalibrator.h
 *
 *  Created on: Nov 21, 2012
 *      Author: lars
 */

#ifndef CAMERACALIBRATOR_H_
#define CAMERACALIBRATOR_H_

#include "camera_capture.h"
#include "calibration_settings.h"
#include "calibrator.h"

class CameraCalibrator : public Calibrator {
 public:
  CameraCalibrator();
  CameraCalibrator(CameraCapture cap, CalibrationSettings s);
//  CameraCalibrator(const CalibrationSettings& settings)
//      : settings(settings) {
//  }
  virtual ~CameraCalibrator();
  void start_();
 protected:
  bool add_sample_(Mat& image);
  double compute_reprojection_errors_();
  bool calibrate_();
  void save_camera_params_();
  bool calibrate_and_save_();

 private:
  CameraCapture cap;
  CalibrationSettings settings;
  clock_t prev_timestamp;

  string input_filename;
  string output_filename;
  vector<vector<Point2f> > image_points;
  vector<vector<Point3f> > object_points;
}
;

#endif /* CAMERACALIBRATOR_H_ */
