# -*- coding: utf-8 -*-
"""
Created on Mon Nov 05 15:21:33 2012

@author: Lars
"""

import cv2
import numpy as np

import math3d as m3d

import threading
import socket
import time

class TurntableCalibrator(object):
    
    def __init__(self, capture, camera_matrix, dist_coeffs, homography, rvec, tvec):
        self.capture = capture        
        
        self.cm = camera_matrix
        self.dc = dist_coeffs
        self.homography = homography
        self.rvec = rvec
        self.tvec = tvec
        
        self._pattern_size = (8,6)
        self._square_size = 0.011
        self._image_size = None
        self._image_points = []
        self._object_points = []
        
        self._circle_points = []
        
        self.timed_frames = []
        
        self.img_col = None
        
        self.win_name = 'Turntable calibration'
#        cv2.namedWindow(self.win_name)
#        cv2.setMouseCallback(self.win_name, self._on_mouse)

    def capture_stamped_images(self):
        for i in range(3):
            t = time.time()
            frame = self.capture.read()
            self.timed_frames.append((t, frame))
            time.sleep(2)
            print(t)
        
    def _on_mouse(self, event, x, y, flags, param):
        x, y = np.int16([x, y])
        if event == cv2.EVENT_LBUTTONDOWN:
            cv2.circle(self.img_col, (int(x), int(y)), 15, (255,0,0), 2)
            self.img_pts_mouse.append(np.array([x,y]))
            print([x,y])
            
    def get_points(self):
        for i in range(3):
            print('center{0}.tif'.format(i+1))
            self.img_col = cv2.imread('center{0}.tif'.format(i+1), 0)
            cv2.imshow(self.win_name, self.img_col)
            cv2.waitKey()
            
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
#        print(pattern_points)
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
        
    def get_outer_corners(self, corners):
        tl = corners[0,0]
        tr = corners[0,-1]
        bl = corners[-1,0]
        br = corners[-1,-1]
        if tl[0] > tr[0]:
                tr,tl = tl,tr
                br,bl = bl,br
        if tl[1] > bl[1]:
                bl,tl=tl,bl
                br,tr=tr,br
        return (tl,tr,bl,br)
        
    def add_sample(self, image, debug=False):
        if not self._image_size:
            self._image_size = image.shape[:2]
        found, corners = self.get_chessboard_corners(image, self._pattern_size)
        if found:
            if len(self._image_points) == 0:
                self._image_points.append(corners)
                rvec, tvec = self.get_pose(self.create_pattern_points(), corners)
                self._circle_points.append(tvec.reshape(3))
            else:
                flipped_corners = self.identify_flip(corners)
                self._image_points.append(flipped_corners)
                rvec, tvec = self.get_pose(self.create_pattern_points(), flipped_corners)
                self._circle_points.append(tvec.reshape(3))
            if debug:
                color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                cv2.drawChessboardCorners(color_image, self._pattern_size, corners, found)
                cv2.imshow(self.win_name, color_image)
                cv2.waitKey(2000)
                print('Next chessboard')
        else:
            cv2.imshow(self.win_name, image)
            cv2.waitKey()
            print('Chessboard not found')
        return found
        
    def identify_flip(self, corners):
        # TODO does not work
        ## WORKAROUND: START APPLICATION WHEN THE ASTERISK IS ON THE BOTTOM
        ## OF THE IMAGE THIS ENSURES THAT THE CORNER POINTS DOES NOT FLIP
#        ip = self._image_points[0].reshape(-1,2)
#        cp = corners.reshape(-1, 2)
#        sip = np.sign(ip[0] - ip[-1])
#        scp = np.sign(cp[0] - cp[-1])
#        print(sip, scp)
#        if not sip.tolist() == scp.tolist():
#            corners = corners[::-1]
#            print('flipped')
        return corners
            
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

    def find_rotation_point_and_axis(self):

        p1 = self._circle_points[0]
        p2 = self._circle_points[1]
        p3 = self._circle_points[2]
        
        t1 = self.timed_frames[0][0]
        t2 = self.timed_frames[1][0]
        t3 = self.timed_frames[2][0]
       
        p1p2 = p2 - p1
        p2p3 = p3 - p2
        
        n = np.cross(p1p2, p2p3)
        n /= np.linalg.norm(n)
        
        P0 = p1 + p1p2 / 2
        Q0 = p2 + p2p3 / 2
        
        u_ = np.cross(p1p2, n)
        u_ /= np.linalg.norm(u_)
        v_ = np.cross(p2p3, n)
        v_ /= np.linalg.norm(v_)
        w0_ = P0 - Q0
        
        a = np.dot(u_, u_)
        b = np.dot(u_, v_)
        c = np.dot(v_, v_)
        d = np.dot(u_, w0_)
        e = np.dot(v_, w0_)
        
        sc = (b*e - c*d) / (a*c - b*b)
        tc = (a*e - b*d) / (a*c - b*b)
        
        c = P0 + sc*u_
        
        v1 = m3d.Vector(p1 - c)
        v2 = m3d.Vector(p2 - c)
        v3 = m3d.Vector(p3 - c)
        
        w1 = v1.angle(v2) / (t2 - t1)
        w2 = v2.angle(v3) / (t3 - t2)
        w = 0.5 * (w1 + w2)
        
        return c, n, w
        
    def run(self, debug=False):
        self.capture_stamped_images()
        for i in range(3):
            ret, frame = self.timed_frames[i][1]
            self.add_sample(frame, debug)
        cv2.destroyAllWindows()
        center, axis, angspeed = tc.find_rotation_point_and_axis()
        print center, axis, angspeed
        
class ATC4Capture(threading.Thread):
    def __init__(self):

        # Socket
        self.host = "localhost"
        self.port = 9999
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.connect((self.host, self.port))

    def read(self):
        recv = ""
        self.sock.send("s")  # s = start
        while(len(recv) < 2048*1088):
            recv += self.sock.recv(8192)
        frame = np.frombuffer(recv, np.uint8)
        frame.resize(1088, 2048)
        time.sleep(0.1)
        return True, frame
        
        


if __name__ == '__main__':
    cm = np.load('../params/cm.npy')
    dc = np.load('../params/dc.npy')
    h = np.load('../params/homography.npy')
    rvec = np.load('../params/rvec_laser_calib_obj.npy')
    tvec = np.load('../params/tvec_laser_calib_obj.npy')
    
    cap = ATC4Capture()
    
    tc = TurntableCalibrator(cap, cm, dc, h, rvec, tvec)
    tc.run(debug=True)
#    tc.capture_stamped_images()
#    for i in range(3):
#        print('center{0}.tif'.format(i+1))
#        img = cv2.imread('center{0}.tif'.format(i+1), 0)
#        tc.add_sample(img) 
#    center, axis, angspeed = tc.find_rotation_point_and_axis()
#    print center, axis, angspeed
        
    
        
        
        

        


    
    

    

    
    
    