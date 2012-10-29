import cv2
import numpy as np



class CameraCalibration(object):

    def __init__(self, pattern_size, square_size):
        self._pattern_size = pattern_size
        self._square_size = square_size
        self._image_size = None
        self._image_points = []
        self._object_points = []

        ## Calibration params
        self._cm = None  # camera matrix
        self._dc = None  # distortion_coefficients
        self._rvecs = None  # rotation vectors
        self._tvecs = None  # translation vectors

        self.calibrated = False

    def create_pattern_points(self):
        """
        Creates chessboard pattern points
        param pattern_size, (int, int): A tuple containing the number of
            chessboard corners,e.g. (8,6)
        param square_size, float: The size of the chessboard squares,
            e.g. 0.035 [m]
        return pattern_points, numpy ndarray: A vector of vectors containing
            the point coordinates for the chessboard corners
        """
        pattern_points = np.zeros( (np.prod(self._pattern_size), 3), np.float32 )
        pattern_points[:,:2] = np.indices(self._pattern_size).T.reshape(-1, 2)
        pattern_points *= self._square_size
        return pattern_points

    def get_chessboard_corners(self, image, pattern_size):
        """
        Returns the chessboard corners in the image
        param image, numpy ndarray
        param pattern_size, (int, int): A tuple containing the number of
            chessboard corners,e.g. (8,6)
        return found, bool: Returns True if the corners were found, else False
        return corners, numpy ndarray: A vector of vectors containing
            the image coordinates for the found chessboard corners
        """
        found, corners = cv2.findChessboardCorners(image, pattern_size,
                flags=(cv2.CALIB_CB_ADAPTIVE_THRESH +
                    cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK))
        if found:
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            cv2.cornerSubPix(image, corners, (11, 11), (-1, -1), term)
            corners.reshape(-1, 2)
        return found, corners

    def fast_check_chessboard(self, image, pattern_size):
        found, corners = cv2.findChessboardCorners(image, pattern_size,
                flags=cv2.CALIB_CB_FAST_CHECK)
        return found

    def add_sample(self, image, debug=False):
        if not self._image_size:
            self._image_size = image.shape[:2]
        found, corners = self.get_chessboard_corners(image, self._pattern_size)
        if found:
            self._image_points.append(corners.reshape(-1,2))
            self._object_points.append(self.create_pattern_points())
            if debug:
                color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                cv2.drawChessboardCorners(color_image, self._pattern_size, corners, found)
                cv2.imshow('DEBUG', color_image)
                cv2.waitKey()                
        else:
            cv2.imshow('', image)
            cv2.waitKey()
            print('Chessboard not found')
        return found

    def calibrate_camera(self):
         calibration = cv2.calibrateCamera(self._object_points, self._image_points, self._image_size)
         ret, self._cm, self._dc, self._rvecs, self._tvecs = calibration
         self.calibrated = True
         print('Calibration finished, RMS: {RMS}'.format(RMS=ret))

    def save_camera_calibration(self, path):
        if self.calibrated:
            np.save('{path}/camera_matrix.npy'.format(path=path), self._cm)
            np.save('{path}/distortion_coefficients.npy'.format(path=path), self._dc)
            np.save('{path}/rvecs.npy'.format(path=path), self._rvecs)
            np.save('{path}/tvecs.npy'.format(path=path), self._tvecs)
            print('Saved calibration to path: {path}'.format(path=path))
        else:
            print('The camera is not calibrated')

if __name__ == '__main__':
    from glob import glob
    img_mask = '../test/chess*.tif'
    img_names = glob(img_mask)
    cc = CameraCalibration((13,13), 0.005)
    for name in img_names:
        frame = cv2.imread(name, 0)
        if cc.add_sample(frame, True):    
            print('Added sample: {name}'.format(name=name))
        else:
            print('Sample was not added')
    cc.calibrate_camera()
    cc.save_camera_calibration(path='../params')
    
    