import cv2
import numpy as np
import pylab
import socket
import struct

import time


blue = (255,0,0)
green = (0,255,0)
red = (0,0,255)
white = (255,255,255)

class LaserCalibration(object):

    def __init__(self, camera_matrix, dist_coeffs, save_path):
        self.cm = camera_matrix
        self.dc = dist_coeffs
        self.homography = None
        self.rvec = None
        self.tvec = None

        self.save_path = save_path

        self.obj_pts = [[0.045, 0.050], [0.015, 0.090],
                        [-0.015, 0.090], [-0.045, 0.050]]
        self.obj_pts_3d = [[0.045, 0.0, 0.050], [0.015, 0.0, 0.090],
                        [-0.015, 0.0, 0.090], [-0.045, 0.0, 0.050]]

        self.img_pts = []
        self.img_col = None
        self.samples = []
        self.calibrated = False

        self.HOST = "localhost"
        self.PORT = 8888
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.linescan_struct = struct.Struct(2048*'H')

        self.fig = pylab.figure(1)
        self.ax = self.fig.add_subplot(111)
        self.ax.grid(True)
        self.ax.set_title('Laser calibration')
        self.ax.axis([0, 2048, 1088, 0])
        self.x_range = pylab.arange(0, 2048, 1)
#        self.line1, = self.ax.plot(2048, 1088)
        self.line2, = self.ax.plot(2048, 1088)
        self.manager = pylab.get_current_fig_manager()
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.timer = self.fig.canvas.new_timer(interval=20)
        self.timer.add_callback(self.real_time_plotter, ())
        self.timer.start()

        print('Laser calibration')
        print('Press MBM to add point, press RBM to calibrate and save calibration')

        pylab.show()

    def on_click(self, event):
        if len(self.img_pts) < 4:
            if event.button == 2:
                self.img_pts.append([event.xdata, event.ydata])
                print('Added point: {x},{y}'.format(x=event.xdata, y=event.ydata))
        else:
            if event.button == 3:
                self.calibrate_laser()
                self.save_laser_calibration()
            if event.button == 2:
                print('All image points added')

    def real_time_plotter(self, arg):
        self.sock.sendto(" ", (self.HOST, self.PORT))
        received = self.sock.recv(4096)
        rec = np.array(self.linescan_struct.unpack(received), np.float64)
        rec /= 2**6  # COG 6bit mode
        xarr, yarr = self.undistort_points(rec)
#        self.line1.set_data(self.x_range, rec)  # Plot raw pixels
        self.line2.set_data(xarr, yarr)  # Plot undistorted pixels
        self.manager.canvas.draw()

    def get_pose(self, object_points, image_points, ransac=False):
        ip = np.array(image_points, np.float32)
        op = np.array(object_points, np.float32)
#        op_3d = np.array([o + [0.0] for o in object_points], np.float32)
#        flag = cv2.CV_P3P
        flag = cv2.CV_ITERATIVE
        if ransac:
            rvec, tvec, inliers = cv2.solvePnPRansac(op, ip, self.cm, self.dc, flags=flag)
            return rvec, tvec
        else:
            ret, rvec, tvec = cv2.solvePnP(op, ip, self.cm, self.dc, flags=flag)
            return rvec, tvec

    def undistort_points(self, scanline):
        length = len(scanline)
        # Create temp array
        temp = np.zeros((length,1,2), dtype=np.float32)
        # Copy scanline into temp array
        for i in range(length):
            temp[i][0][0] = i
            temp[i][0][1] = scanline[i]
        # Undistort and reproject points to pixel values by setting
        # P = camera_matrix
        ud = cv2.undistortPoints(temp, self.cm, self.dc, P=self.cm)
        # Resize array to shape (-1, 2)
        ud.resize((length,2))
        # Split array columnwise
        x, y = np.hsplit(ud, 2)
        return x, y

    def get_homography(self, object_points, image_points):
        ip = np.array(image_points, np.float32)
        op = np.array(object_points, np.float32)
        homography, mask = cv2.findHomography(ip, op)
        return homography, mask

    def calibrate_laser(self):
        self.homography, m = self.get_homography(self.obj_pts, self.img_pts)
        self.rvec, self.tvec = self.get_pose(self.obj_pts_3d, self.img_pts, ransac=False)
        print('Homography')
        print(self.homography)
        print('Rotation matrix')
        rm, jac = cv2.Rodrigues(self.rvec)
        print(rm)
        print('Translation vector')
        print(self.tvec)
        self.calibrated = True

    def save_laser_calibration(self):
        if self.calibrated:
            np.save('{path}/homography.npy'.format(path=self.save_path),
                self.homography)
            np.save('{path}/rvec_laser_calib_obj.npy'.format(path=self.save_path),
                self.rvec)
            np.save('{path}/tvec_laser_calib_obj.npy'.format(path=self.save_path),
                self.tvec)
            print('Saved calibration to path: {path}'.format(path=self.save_path))
        else:
            print('Laser is not calibrated!')


if __name__ == '__main__':
    cm = np.load('../params/cm.npy')
    dc = np.load('../params/dc.npy')


    lc = LaserCalibration(cm, dc, '../params')




