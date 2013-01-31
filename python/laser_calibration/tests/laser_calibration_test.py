# -*- coding: utf-8 -*-
"""
Created on Thu Jan 31 08:51:47 2013

@author: Lars
"""

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import numpy as np
import cv2

class LaserCalibrationSteppedObjectTest(object):
    
    def __init__(self, cm, dc, c2w_rmat, c2w_tvec, ll2d, ll3d, lplane, lpoint):
        self._cm = cm
        self._dc = dc
        self._c2w_rmat = c2w_rmat
        self._c2w_tvec = c2w_tvec
        self._ll2d = ll2d
        self._ll3d = ll3d
        self._lplane = lplane
        self._lpoint = lpoint        
    
    def intersect_with_laser_plane(self):
        ## resize array to get into correct shape for cv2.undistortPoints
        ll2d = self._ll2d.copy()  
        ll2d.resize(self._ll2d.shape[0], 1, 2)

        ## undistort and normalize
        ud = cv2.undistortPoints(ll2d, self._cm, self._dc)
        ud.resize(ll2d.shape[0], 2)
        ud_h = np.hstack((ud, np.ones((ud.shape[0], 1))))
        
        # http://en.wikipedia.org/wiki/Line-plane_intersection        
        
        w2c_rmat = self._c2w_rmat.T
        p3d_array = []
        for udp in ud_h:
            pw = np.dot(w2c_rmat, udp.reshape(3,1) - c2w_tvec)
            l0 = np.dot(w2c_rmat, -c2w_tvec)
            l0 = l0.reshape(3)
            l = pw.reshape(3) - l0.reshape(3)
            ll = np.linalg.norm(l)
            if ll != 0:
                l /= ll
                

            n = self._lplane.reshape(3)
            p0 = self._lpoint.reshape(3)
            l = l.reshape(3)
            
            d1 = np.dot((p0 - l0), n)
            d2 = np.dot(l.reshape(3), n)
            
            p3d = (d1/d2)*l + l0 
            p3d_array.append(p3d)
            
            
        self.p3d_array = np.array(p3d_array)
        
        return self.p3d_array
        
        
    def plot3d(self, p3d1, p3d2=None):
#        p3d = self.intersect_with_laser_plane()
        xs, ys, zs = np.hsplit(p3d1, 3)
        xs = xs.reshape(xs.shape[0])
        ys = ys.reshape(ys.shape[0])
        zs = zs.reshape(ys.shape[0])

        mpl.rcParams['legend.fontsize'] = 10

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot(xs, ys, zs, label='3d plot of laserline')
        if p3d2 is not None:
            xs2, ys2, zs2 = np.hsplit(p3d2, 3)
            xs2 = xs2.reshape(xs2.shape[0])
            ys2 = ys2.reshape(ys2.shape[0])
            zs2 = zs2.reshape(zs2.shape[0])
            ax.plot(xs2, ys2, zs2, label='3d plot of laserline #2')
            
        ax.legend()
        
#        plt.show()


    def show(self):
        plt.show()
        
            
            
        
        
        
    




if __name__ == '__main__':
    
    cm = np.load('../stepped_calibration_object/cm.npy')
    dc = np.load('../stepped_calibration_object/dc.npy')
    c2w_rmat = np.load('../stepped_calibration_object/c2w_rotmatrix.npy')
    c2w_tvec = np.load('../stepped_calibration_object/c2w_transvector.npy')
    ll2d = np.loadtxt('../stepped_calibration_object/laserline_2d.txt', dtype=np.float32)
    ll3d = np.loadtxt('../stepped_calibration_object/laserline_3d_in_object.txt', dtype=np.float32)
    lplane = np.loadtxt('../stepped_calibration_object/laser_plane.txt', dtype=np.float32)
    lpoint = np.loadtxt('../stepped_calibration_object/point_on_laser_plane.txt', dtype=np.float32)
    
    
    test_obj = LaserCalibrationSteppedObjectTest(cm, dc, c2w_rmat, c2w_tvec, ll2d, ll3d, lplane, lpoint)
    test_obj.plot3d(test_obj.intersect_with_laser_plane(), ll3d)
#    test_obj.plot3d(ll3d)
    test_obj.show()
    
    
    
    