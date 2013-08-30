/*
 * scanner_py.h
 *
 *  Created on: May 30, 2013
 *      Author: lars
 */

#ifndef SCANNER_PY_H_
#define SCANNER_PY_H_

#include <string>
#include "scanner.h"

namespace bp = boost::python;
namespace np = boost::numpy;

namespace laser_scanning_library {

class ScannerPy {
 public:
  ScannerPy(np::ndarray cm, np::ndarray dc, np::ndarray rmat, np::ndarray tvec,
            np::ndarray lplane, np::ndarray lpoint)
      : scanner_(ndarray2Mat(cm), ndarray2Mat(dc), ndarray2Mat(rmat),
                 ndarray2Mat(tvec), ndarray2Mat(lplane), ndarray2Mat(lpoint)),
        p3ds_(2048 * 3),
        xrange_(2048, 1, CV_32F) {
    // initialize matrices
//    init();
  }

  virtual ~ScannerPy();
  np::ndarray get3DCoords(np::ndarray arr);
 private:
  Mat ndarray2Mat(np::ndarray arr);
//  void init();
  std::vector<double> p3ds_;
  Mat xrange_;
  Scanner scanner_;
};

} /* namespace laser_scanning_library */
#endif /* SCANNER_PY_H_ */
