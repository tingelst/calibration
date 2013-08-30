/*
 Base class for intrinsic and extrinsic calibration using OpenCV
 Copyright (C) 2012  Lars Tingelstad <lars.tingelstad@ntnu.no>

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CALIBRATOR_H_
#define CALIBRATOR_H_

// Standard includes
#include <vector>
// OpenCV includes
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

class Calibrator {
 public:
  Calibrator();
  virtual ~Calibrator();
  static void calc_chessboard_corners_(Size board_size, float square_size,
                                       vector<Point3f>& corners);
  Mat get_camera_matrix() {
    return this->camera_matrix;
  }
  Mat get_dist_coeffs() {
    return this->dist_coeffs;
  }
  bool read_intrinsics_(const string filename);
  bool write_intrinsics_(const string filename);
  void set_board_size(Size board_size) {
    this->board_size = board_size;
  }
  Size get_board_size_() {
    return this->board_size;
  }
  void set_square_size_(float square_size) {
    this->square_size = square_size;
  }
  float get_square_size_() {
    return this->square_size;
  }

 protected:
  enum { CAPTURING = 0, CALIBRATED = 1 };
  Mat camera_matrix;
  Mat dist_coeffs;
  Size image_size;  // Imagesize w x h
  Size board_size;  // The size of the board -> Number of items by width and height
  float square_size;  // The size of a square in your defined unit (point, millimeter,etc).
  vector<Mat> rvecs, tvecs;
  vector<float> reproj_errs;
  vector<float> per_view_errors;
  double total_avg_err;
  int flags;
  float aspect_ratio;
};

#endif /* CALIBRATOR_H_ */
