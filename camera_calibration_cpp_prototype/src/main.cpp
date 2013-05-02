// Standard includes
#include <iostream>
#include <stdio.h>
#include <string>

// Local includes
#include "calibration_settings.h"
#include "camera_calibrator.h"
#include "camera_capture.h"

using namespace cv;
using namespace std;

static void read(const FileNode& node, CalibrationSettings& x,
                 const CalibrationSettings& default_value =
                     CalibrationSettings()) {
  if (node.empty())
    x = default_value;
  else
    x.read(node);
}

int main(int argc, char **argv) {


  // Camera
  CameraCapture cap = CameraCapture();

  // Calibration settings
  CalibrationSettings cal_settings;
  const string input_settings_file = argc > 1 ? argv[1] : "default.xml";
  FileStorage fs(input_settings_file, FileStorage::READ);
  if (!fs.isOpened()) {
    cout << "Could not open the configuration file: \"" << input_settings_file
         << "\"" << endl;
    return -1;
  }

  fs["settings"] >> cal_settings;
  fs.release();

  if (!cal_settings.good_input) {
    cout << "Invalid input detected. Application stopping. " << endl;
    return -1;
  }

  // Camera calibrator
  CameraCalibrator cam_cal = CameraCalibrator(cap, cal_settings);
  // Start calibration
  cam_cal.start_();



  return 0;

}
