// Standard includes
#include <iostream>
#include <stdio.h>
#include <string>

// OpenCV includes
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

// Local includes
#include "lasercalibrator.h"
#include "cameracalibrator.h"
#include "camera_capture.h"

using namespace cv;
using namespace std;

int main(int argc, char **argv) {

	LaserCalibrator laser_cal = LaserCalibrator();
	if (laser_cal.read_intrinsics_("../out_camera_data.yml"))
		laser_cal.write_intrinsics_("../test.yml");
	cout << laser_cal.get_camera_matrix() << endl;
	laser_cal.set_board_size(Size(8, 6));
	CameraCapture cc = CameraCapture();

	string window_name("display");
	namedWindow(window_name);
	Mat image;

	vector<Point2f> point_buf;
	bool found;

	for (;;) {
		cc.read_(image);

		bitwise_not(image, image);

		imshow(window_name, image);
		char c = waitKey(30);
		if (c == 27)
			break;
	}

	return 0;

}
