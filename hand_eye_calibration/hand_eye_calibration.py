
from park_martin_calibration import ParkMartinCalibrator
from tsai_lenz_calibration import TsaiLenzCalibrator
import cv2
import numpy as np
import math3d as m3d

class HandEyeCalibration(object):

    def __init__(self, pattern_size, square_size, camera_matrix, dist_coefs, debug=False):
        self._pattern_size = pattern_size
        self._square_size = square_size
        self._camera_matrix = camera_matrix
        self._dist_coefs = dist_coefs
        self._debug = debug

        #self.calibrator = ParkMartinCalibrator()
        self.calibrator = TsaiLenzCalibrator()

        self._pose_pairs = []

    def _get_image(self):
        image = self._camera.capture()
        return image

    def _get_object_points(self):
        pattern_size = self._pattern_size
        pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 )
        pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
        pattern_points *= self._square_size
        return pattern_points

    def _get_chessboard_corners(self, image):

        h, w = image.shape[:2]
        found, corners = cv2.findChessboardCorners(image, self._pattern_size,
                                                   flags=(cv2.CALIB_CB_ADAPTIVE_THRESH
                                                          + cv2.CALIB_CB_NORMALIZE_IMAGE
                                                          + cv2.CALIB_CB_FAST_CHECK))
        if found:
            term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
            cv2.cornerSubPix(image, corners, (5, 5), (-1, -1), term)
            corners.reshape(-1, 2)
            if not found:
                print '[HEC] Chessboard not found'
            if self._debug:
                color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                cv2.drawChessboardCorners(color_image,
                                          self._pattern_size,
                                          corners,
                                          found)
                cv2.imshow('image', color_image)
                cv2.waitKey()

        return corners

    def get_object_pose(self, image_points):
        object_points = self._get_object_points()
        ret, rvec, tvec = cv2.solvePnP(object_points, image_points,
                                       self._camera_matrix, self._dist_coefs)
        pose = rvec, tvec
        return pose


    def get_m3d_pose(self, obj_pose):
        obj_pose = m3d.Transform(obj_pose[0].reshape(3), obj_pose[1].reshape(3))
        return obj_pose

if __name__ == '__main__':

    #import pymoco.client_utils as pcu
    import sys

    pattern_size = (15,11)
    square_size = 50

    camera_matrix = np.load('camera_calibration/cm.npy')
    dist_coefs = np.load('camera_calibration/dc.npy')

    if 'debug' in sys.argv:
        hec = HandEyeCalibration(pattern_size, square_size,
                                  camera_matrix, dist_coefs, debug=True)
    else:
        hec = HandEyeCalibration(pattern_size, square_size,
                                 camera_matrix, dist_coefs)


    pose_pairs = []

    import scipy.io as sio
    b2h_poses = sio.loadmat('hec_benchmark_dataset/RobotPoses/T_base_hand.mat')
    b2h_poses = b2h_poses['T_base_hand'].reshape(25)

    from glob import glob
    img_mask = 'hec_benchmark_dataset/Scenario1/imagePhysical_*.png'

    img_names = sorted(glob(img_mask))

    for i in range(12,24):
        img_name = img_names[i]
        b2h = b2h_poses[i]
        image = cv2.imread(img_name, 0)
        image_points = hec._get_chessboard_corners(image)
        c2o = hec.get_object_pose(image_points)
        c2o_m3d = hec.get_m3d_pose(c2o).inverse()
        np.savetxt('benchmark_transforms/o2c_{0}.txt'.format(i), c2o_m3d.data)
        #print("--------------------------------- " + str(i))
        #print(c2o_m3d)
        b2h_m3d = m3d.Transform(b2h)
        np.savetxt('benchmark_transforms/h2b_{0}.txt'.format(i), b2h_m3d.data)
        #print('b2h_{0}'.format(i))
        #print b2h_m3d
        pose_pairs.append((b2h_m3d, c2o_m3d))
        i += 1

    pp = np.array(pose_pairs)

    hec.calibrator.pose_pairs = pp[:10]
    #print(hec._pmc.pose_pairs)
    sif_cal = hec.calibrator.sensor_in_flange
    print('Sensor in flange:')
    print(sif_cal)

