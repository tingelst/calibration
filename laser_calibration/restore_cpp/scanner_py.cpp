/*
 * scanner_py.cpp
 *
 *  Created on: May 30, 2013
 *      Author: lars
 */

#include <iostream>

#include <boost/python.hpp>
#include <boost/numpy.hpp>

#include "scanner_py.h"

namespace bp = boost::python;
namespace np = boost::numpy;

using namespace std;

namespace laser_scanning_library {

ScannerPy::~ScannerPy() {
    // TODO Auto-generated destructor stub
}

//void ScannerPy::init() {
//  vector<float> range;
//  for (float i = 0; i < 2048; i++)
//    range.push_back(i);
//  xrange_ = Mat(range);
//}

np::ndarray ScannerPy::get3DCoords(np::ndarray arr) {
    Mat laserline = ndarray2Mat(arr);

    Mat laserline2(2048, 2, CV_32F);
    Mat xrange(2048, 1, CV_32F);
    vector<float> range;
    for (float i = 0; i < 2048; i++)
        range.push_back(i);
    xrange = Mat(range);

    xrange.col(0).copyTo(laserline2.col(0));
    laserline2.col(1) = laserline.col(0) / 64.0;
    laserline2 = laserline2.reshape(2, 0);

    p3ds_ = scanner_.get3DCoords(laserline2);

    int cols = 3;
    int rows = 2048;

    bp::tuple shape = bp::make_tuple(2048, 3);
    bp::tuple strides = bp::make_tuple(3 * sizeof(double), sizeof(double));

    np::ndarray ret = np::from_data(&p3ds_[0], np::dtype::get_builtin<double>(), shape, strides, bp::object());
//  std::cout << "Original array:\n" << bp::extract<char const *>(bp::str(b)) << std::endl;

    return ret;
}

Mat ScannerPy::ndarray2Mat(np::ndarray inarray) {

    if (!(inarray.get_flags() & np::ndarray::C_CONTIGUOUS)) {
        throw "ScannerPy::ndarray2Mat input array must be contiguous";
    }
    int rows = inarray.shape(0);
    int cols = inarray.shape(1);
    return Mat(rows, cols, CV_32F, reinterpret_cast<void*>(inarray.get_data()));
}

} /* namespace laser_scanning_library */

namespace lsl = laser_scanning_library;

BOOST_PYTHON_MODULE(liblaser_scanner_py)
{
    np::initialize();

    bp::class_<lsl::ScannerPy> scanner(
        "Scanner", bp::init<np::ndarray, np::ndarray, np::ndarray, np::ndarray, np::ndarray, np::ndarray>());
    scanner.def("get_3d_coords", &lsl::ScannerPy::get3DCoords);
}
