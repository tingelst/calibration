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
        
        self.save_path = save_path
        
        self.obj_pts = [[0.045, 0.050], [0.015, 0.090], 
                        [-0.015, 0.090], [-0.045, 0.050]]
                           
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
        self.line1, = self.ax.plot(2048, 1088)
        self.manager = pylab.get_current_fig_manager()
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.timer = self.fig.canvas.new_timer(interval=20)
        self.timer.add_callback(self.real_time_plotter, ())
        self.timer.start()
        
        print('Laser calibration')
        print('Press MBM to add point, press RBM to calibrate and save calibration')

        pylab.show()
         
#        self.win_name = 'Laser calibration'
#        cv2.namedWindow(self.win_name)
#        cv2.setMouseCallback(self.win_name, self._on_mouse)

        
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
        rec = np.array(self.linescan_struct.unpack(received))
        self.line1.set_data(self.x_range, rec)
        self.manager.canvas.draw()

        
            
    
#    def _on_mouse(self, event, x, y, flags, param):
#        x, y = np.int16([x, y])
#        if event == cv2.EVENT_LBUTTONDOWN:
#            cv2.circle(self.img_col, (int(x), int(y)), 15, red, 2)
#            self.img_pts.append([x,y])
            
#    def _img_from_scanline(self, scan_line):
#        img = np.zeros((1088, 2048), np.uint8)
#        for i in range(len(scan_line)):
#            img[scan_line[i], i] = 255
#        return img

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
    
    def save_laser_calibration(self):
        if self.calibrated:
            np.save('{path}/homography.npy'.format(path=self.save_path), self.homography)
            print('Saved calibration to path: {path}'.format(path=self.save_path))
        else:
            print('Laser is not calibrated!')


if __name__ == '__main__':
    cm = np.load('../params/camera_matrix.npy')
    dc = np.load('../params/distortion_coefficients.npy')
    scan_line = np.load('../test/calibobj.npy')

    lc = LaserCalibration(cm, dc, '../params')

    
    
    
