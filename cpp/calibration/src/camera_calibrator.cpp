/*
 * cameracalibrator.cpp
 *
 *  Created on: Nov 21, 2012
 *      Author: lars
 */
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "camera_calibrator.h"

using namespace cv;
using namespace std;

CameraCalibrator::CameraCalibrator() {

}

CameraCalibrator::CameraCalibrator(CameraCapture c,  CalibrationSettings s)
    : cap(c), settings(s) {

}

CameraCalibrator::~CameraCalibrator() {
}

void CameraCalibrator::start_() {

  const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
  const char ESC_KEY = 27;

  string window_name("display");
  namedWindow(window_name);
  int mode = CAPTURING;
  for (int i;; ++i) {

    Mat image;
    bool blink_output = false;

    cap.read_(image);
    bool found;
    if (add_sample_(image)) {
      blink_output = true;
    }

    char key = (char) waitKey(30);
    if (key == ESC_KEY)
      break;

    if (key == 'c')
      calibrate_and_save_();

    if (blink_output)
      bitwise_not(image, image);

    //----------------------------- Output Text ------------------------------------------------
    string msg(
        mode == CAPTURING ? "100/100" :
        mode == CALIBRATED ? "Calibrated" : "Press 'g' to start");
    int baseline = 0;
    Size text_size = getTextSize(msg, 1, 1, 1, &baseline);
    Point text_origin(image.cols - 2 * text_size.width - 10,
                      image.rows - 2 * baseline - 10);
    putText(image, msg, text_origin, 1, 1, mode == CALIBRATED ? GREEN : RED);

    //------------------------------ Show image and check for input commands -------------------

    imshow(window_name, image);


  }


}

bool CameraCalibrator::add_sample_(Mat& image) {
  vector<Point2f> point_buf;
  bool found;
  found = findChessboardCorners(
      image,
      settings.board_size,
      point_buf,
      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK
          | CV_CALIB_CB_NORMALIZE_IMAGE);
  if (found) {
    Mat viewGray;
    cvtColor(image, viewGray, CV_BGR2GRAY);
    cornerSubPix(viewGray, point_buf, Size(11, 11), Size(-1, -1),
                 TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    // For camera only take new samples after delay time
    if (clock() - prev_timestamp > settings.delay * 1e-3 * CLOCKS_PER_SEC) {
      // Draw the corners.
      drawChessboardCorners(image, settings.board_size, Mat(point_buf), found);
      image_points.push_back(point_buf);

    }

    cout << "Image points: " << image_points.size() << endl;

  }
  return found;
}

double CameraCalibrator::compute_reprojection_errors_() {
  vector<Point2f> image_points2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  per_view_errors.resize(object_points.size());

  for (i = 0; i < (int) object_points.size(); ++i) {
    projectPoints(Mat(object_points[i]), rvecs[i], tvecs[i], camera_matrix,
                  dist_coeffs, image_points2);
    err = norm(Mat(image_points[i]), Mat(image_points2), CV_L2);

    int n = (int) object_points[i].size();
    per_view_errors[i] = (float) std::sqrt(err * err / n);
    totalErr += err * err;
    totalPoints += n;
  }

  return std::sqrt(totalErr / totalPoints);
}

bool CameraCalibrator::calibrate_() {

  camera_matrix = Mat::eye(3, 3, CV_64F);
  if (settings.flag & CV_CALIB_FIX_ASPECT_RATIO)
    camera_matrix.at<double>(0, 0) = 1.0;
  dist_coeffs = Mat::zeros(8, 1, CV_64F);

  vector<vector<Point3f> > object_points(1);
  calc_chessboard_corners_(settings.board_size, settings.square_size,
                           object_points[0]);
  object_points.resize(image_points.size(), object_points[0]);

  cout << "object points: " << object_points.size() << endl;

  //Find intrinsic and extrinsic camera parameters
  double rms = calibrateCamera(
      object_points, image_points, settings.image_size, camera_matrix,
      dist_coeffs, rvecs, tvecs,
      settings.flag | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

  cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

  bool ok = checkRange(camera_matrix) && checkRange(dist_coeffs);

  total_avg_err = compute_reprojection_errors_();

  return ok;
}

// Print camera parameters to the output file
void CameraCalibrator::save_camera_params_() {

  FileStorage fs(settings.output_file_name, FileStorage::WRITE);

  time_t tm;
  time(&tm);
  struct tm *t2 = localtime(&tm);
  char buf[1024];
  strftime(buf, sizeof(buf) - 1, "%c", t2);

  fs << "calibration_time" << buf;

  if (!rvecs.empty() || !reproj_errs.empty())
    fs << "nr_of_frames" << (int) std::max(rvecs.size(), reproj_errs.size());
  fs << "image_width" << image_size.width;
  fs << "image_height" << image_size.height;
  fs << "board_width" << board_size.width;
  fs << "board_height" << board_size.height;
  fs << "square_size" << square_size;

  if (flags & CV_CALIB_FIX_ASPECT_RATIO)
    fs << "fix_aspect_ratio" << aspect_ratio;

  if (flags) {
    sprintf(buf, "flags: %s%s%s%s",
            flags & CV_CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
            flags & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
            flags & CV_CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
            flags & CV_CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "");
    cvWriteComment(*fs, buf, 0);
  }

  fs << "flag_value" << flags;

  fs << "camera_matrix" << camera_matrix;
  fs << "distortion_coefficients" << dist_coeffs;

  fs << "avg_reprojection_error" << total_avg_err;
  if (!reproj_errs.empty())
    fs << "per_view_reprojection_errors" << Mat(reproj_errs);

  if (!rvecs.empty() && !tvecs.empty()) {
    CV_Assert(rvecs[0].type() == tvecs[0].type());
    Mat bigmat((int) rvecs.size(), 6, rvecs[0].type());
    for (int i = 0; i < (int) rvecs.size(); i++) {
      Mat r = bigmat(Range(i, i + 1), Range(0, 3));
      Mat t = bigmat(Range(i, i + 1), Range(3, 6));

      CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
      CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
      //*.t() is MatExpr (not Mat) so we can use assignment operator
      r = rvecs[i].t();
      t = tvecs[i].t();
    }
    cvWriteComment(
        *fs,
        "a set of 6-tuples (rotation vector + translation vector) for each view",
        0);
    fs << "extrinsic_parameters" << bigmat;
  }

  if (!image_points.empty()) {
    Mat image_pt_mat((int) image_points.size(), (int) image_points[0].size(),
                     CV_32FC2);
    for (int i = 0; i < (int) image_points.size(); i++) {
      Mat r = image_pt_mat.row(i).reshape(2, image_pt_mat.cols);
      Mat imgpti(image_points[i]);
      imgpti.copyTo(r);
    }
    fs << "image_points" << image_pt_mat;
  }
}

bool CameraCalibrator::calibrate_and_save_() {

  bool ok = calibrate_();
  cout << (ok ? "Calibration succeeded" : "Calibration failed")
       << ". avg re projection error = " << total_avg_err;

  if (ok)
    save_camera_params_();
  return ok;

}



