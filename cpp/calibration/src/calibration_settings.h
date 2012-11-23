/*
 * calibration_settings.h
 *
 *  Created on: Nov 22, 2012
 *      Author: lars
 */

#ifndef CALIBRATION_SETTINGS_H_
#define CALIBRATION_SETTINGS_H_

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;
using namespace std;

class CalibrationSettings {
 public:
  CalibrationSettings() : good_input(false) {}
  enum Pattern {
    NOT_EXISTING,
    CHESSBOARD,
    CIRCLES_GRID,
    ASYMMETRIC_CIRCLES_GRID
  };

  void write(FileStorage& fs) const         //Write serialization for this class
      {
    fs << "{"
       << "board_size_width"  << board_size.width
       << "board_size_height" << board_size.height
       << "square_size" << square_size
       << "calibrate_pattern" << pattern_to_use
       << "calibrate_nr_of_frame_to_use" << nr_frames
       << "calibrate_fix_aspect_Ratio" << aspect_ratio
       << "calibrate_assume_zero_tangential_distortion" << calib_zero_tangent_dist
       << "calibrate_fix_principal_point_at_the_center" << calib_fix_principal_point
       << "write_detected_feature_points" << write_points
       << "write_extrinsic_parameters" << write_extrinsics
       << "write_outputFileName" << output_file_name
       << "show_undistorted_image" << show_undistorted
       << "Input_Delay" << delay
       << "}";

  }

  void read(const FileNode& node)            //Read serialization for this class
      {
    node["image_size_width"] >> image_size.width;
    node["image_size_height"] >> image_size.height;
    node["board_size_width"] >> board_size.width;
    node["board_size_height"] >> board_size.height;
    node["calibrate_pattern"] >> pattern_to_use;
    node["square_size"] >> square_size;
    node["calibrate_nr_of_frame_to_use"] >> nr_frames;
    node["calibrate_fix_aspect_ratio"] >> aspect_ratio;
    node["write_detected_feature_points"] >> write_points;
    node["write_extrinsic_parameters"] >> write_extrinsics;
    node["write_output_filename"] >> output_file_name;
    node["calibrate_assume_zero_tangential_distortion"] >> calib_zero_tangent_dist;
    node["calibrate_fix_principal_point_at_center"] >> calib_fix_principal_point;
    node["show_undistorted_image"] >> show_undistorted;
    node["input_delay"] >> delay;
    check_settings_();
  }

  void check_settings_() {
    good_input = true;
    if (board_size.width <= 0 || board_size.height <= 0) {
      cerr << "Invalid Board size: " << board_size.width << " "
          << board_size.height << endl;
      good_input = false;
    }
    if (square_size <= 10e-6) {
      cerr << "Invalid square size " << square_size << endl;
      good_input = false;
    }
    if (nr_frames <= 0) {
      cerr << "Invalid number of frames " << nr_frames << endl;
      good_input = false;
    }

    flag = 0;
    if (calib_fix_principal_point)
      flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
    if (calib_zero_tangent_dist)
      flag |= CV_CALIB_ZERO_TANGENT_DIST;
    if (aspect_ratio)
      flag |= CV_CALIB_FIX_ASPECT_RATIO;

    calibration_pattern = NOT_EXISTING;
    if (!pattern_to_use.compare("CHESSBOARD"))
      calibration_pattern = CHESSBOARD;
    if (!pattern_to_use.compare("CIRCLES_GRID"))
      calibration_pattern = CIRCLES_GRID;
    if (!pattern_to_use.compare("ASYMMETRIC_CIRCLES_GRID"))
      calibration_pattern = ASYMMETRIC_CIRCLES_GRID;
    if (calibration_pattern == NOT_EXISTING) {
      cerr << " Inexistent camera calibration mode: " << pattern_to_use << endl;
      good_input = false;
    }

  }

  static bool read_string_list_(const string& filename, vector<string>& l) {
    l.clear();
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
      return false;
    FileNode n = fs.getFirstTopLevelNode();
    if (n.type() != FileNode::SEQ)
      return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for (; it != it_end; ++it)
      l.push_back((string) * it);
    return true;
  }
 public:
  Size image_size; // Image size
  Size board_size;  // The size of the board -> Number of items by width and height
  Pattern calibration_pattern;  // One of the Chessboard, circles, or asymmetric circle pattern
  float square_size;  // The size of a square in your defined unit (point, millimeter,etc).
  int nr_frames;    // The number of frames to use from the input for calibration
  float aspect_ratio;         // The aspect ratio
  int delay;                 // In case of a video input
  bool write_points;         //  Write detected feature points
  bool write_extrinsics;     // Write extrinsic parameters
  bool calib_zero_tangent_dist;  // Assume zero tangential distortion
  bool calib_fix_principal_point;  // Fix the principal point at the center
  string output_file_name;      // The name of the file where to write
  bool show_undistorted;       // Show undistorted images after calibration

  bool good_input;
  int flag;

 private:
  string pattern_to_use;

};

#endif /* CALIBRATION_SETTINGS_H_ */
