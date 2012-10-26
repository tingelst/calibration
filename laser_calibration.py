import cv2
import numpy as np


class LaserCalibration(object):

    def __init__(self, camera_matrix, dist_coeffs):
        self.cm = camera_matrix
        self.dc = dist_coeffs

        print('Laser calibration')

    def get_pose(self, object_points, image_points, ransac=False):
        if ransac:
            rvec, tvec = cv2.solvePnPRansac(object_points, image_points,
                                            self.cm, self.dc)
            return rvec, tvec
        else:
            ret, rvec, tvec = cv2.solvePnP(object_points, image_points,
                                           self.cm, self.dc)
            return rvec, tvec


if __name__ == '__main__':
    cm = np.load('camera_matrix.npy')
    dc = np.load('dist_coeffs.npy')

    lc = LaserCalibration()
