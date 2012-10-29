import cv2
import numpy as np

blue = (255,0,0)
green = (0,255,0)
red = (0,0,255)
white = (255,255,255)

class LaserCalibration(object):

    def __init__(self, camera_matrix, dist_coeffs):
        self.cm = camera_matrix
        self.dc = dist_coeffs
        self.homography = None
        
        self.obj_pts = [[0.045, 0.050], [0.015, 0.090], 
                        [-0.015, 0.090], [-0.045, 0.050]]
                           
        self.img_pts = []
        self.img_col = None
        self.samples = []
        self.calibrated = False
        
        self.win_name = 'Laser calibration'
        cv2.namedWindow(self.win_name)
        cv2.setMouseCallback(self.win_name, self._on_mouse)

    def _on_mouse(self, event, x, y, flags, param):
        x, y = np.int16([x, y])
        if event == cv2.EVENT_LBUTTONDOWN:
            cv2.circle(self.img_col, (int(x), int(y)), 15, red, 2)
            self.img_pts.append([x,y])
            
    def _img_from_scanline(self, scan_line):
        img = np.zeros((1088, 2048), np.uint8)
        for i in range(len(scan_line)):
            img[scan_line[i], i] = 255
        return img
        
    def add_sample(self, scan_line, undistort=False):
        img = self._img_from_scanline(scan_line)
        if undistort:
            sample = cv2.undistort(img, self.cm, self.dc)
        else:
            sample = img
        self.img_col = cv2.cvtColor(sample, cv2.cv.CV_GRAY2BGR)
        while True:
            cv2.imshow(self.win_name, self.img_col)
            c = cv2.waitKey(30)
            if len(self.img_pts) == 4:
                cv2.imshow(self.win_name, self.img_col)
                cv2.waitKey()
                break
            if c == 27:  # ESC
                break

    def get_pose(self, object_points, image_points, ransac=False):
        if ransac:
            rvec, tvec = cv2.solvePnPRansac(object_points, image_points,
                                            self.cm, self.dc)
            return rvec, tvec
        else:
            ret, rvec, tvec = cv2.solvePnP(object_points, image_points,
                                           self.cm, self.dc)
            return rvec, tvec


    def get_homography(self, object_points, image_points):
        ip = np.array(image_points, np.float32)
        op = np.array(object_points, np.float32)
        homography, mask = cv2.findHomography(ip, op)
        return homography, mask
        
    def calibrate_laser(self):
        h, m = self.get_homography(self.obj_pts, self.img_pts)
        self.homography = h
        print(self.homography)
        self.calibrated = True        
    
    def save_laser_calibration(self, path):
        if self.calibrated:
            np.save('{path}/homography.npy'.format(path=path), self.homography)
            print('Saved calibration to path: {path}'.format(path=path))
        else:
            print('Laser is not calibrated!')


if __name__ == '__main__':
    cm = np.load('../params/camera_matrix.npy')
    dc = np.load('../params/distortion_coefficients.npy')
    scan_line = np.load('../params/calibobj.npy')

    lc = LaserCalibration(cm, dc)
    lc.add_sample(scan_line)
    lc.calibrate_laser()
    lc.save_laser_calibration('../params')

    
    
    
