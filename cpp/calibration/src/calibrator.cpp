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

#include <iostream>
#include <time.h>

#include "calibrator.h"

using namespace cv;
using namespace std;


Calibrator::Calibrator() {
  this->flags = 0;

}

Calibrator::~Calibrator() {
  // TODO Auto-generated destructor stub
}

void Calibrator::calc_chessboard_corners_(Size board_size, float square_size,
                                          vector<Point3f>& corners) {
  corners.resize(0);
  for (int i = 0; i < board_size.height; i++)
    for (int j = 0; j < board_size.width; j++)
      corners.push_back(
          Point3f(float(j * square_size), float(i * square_size), 0));
}

bool Calibrator::read_intrinsics_(const string filename) {
  FileStorage fs(filename, FileStorage::READ);
  if (fs.isOpened()) {
    fs["image_width"] >> this->image_size.width;
    fs["image_height"] >> this->image_size.height;
    fs["distortion_coefficients"] >> this->dist_coeffs;
    fs["camera_matrix"] >> this->camera_matrix;

    if (this->dist_coeffs.type() != CV_64F)
      this->dist_coeffs = Mat_<double>(this->dist_coeffs);
    if (this->camera_matrix.type() != CV_64F)
      this->camera_matrix = Mat_<double>(this->camera_matrix);
    return true;
  }
  return false;
}

bool Calibrator::write_intrinsics_(const string filename) {
  FileStorage fs(filename, FileStorage::WRITE);
  if (fs.isOpened()) {
    time_t rawtime;
    time(&rawtime);
    fs << "calibration_date" << asctime(localtime(&rawtime));
    fs << "camera_matrix" << this->camera_matrix;
    fs << "distortion_coefficients" << this->dist_coeffs;
    fs.release();
    return true;
  } else
    return false;

}

