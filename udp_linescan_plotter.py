import time

import socket
import numpy as np
import cv2
import struct
import pylab
from matplotlib.widgets import Button

import math3d as m3d

class UDPLinescanPlotter(object):
    
    def __init__(self, camera_matrix, dist_coeffs, homography, rvec, tvec, angspeed, center, axis):
        self.cm = camera_matrix
        self.dc = dist_coeffs
        self.homography = homography
        self.rvec = rvec
        self.tvec = tvec
        self.angspeed = angspeed
        self.center = center
        self.axis = axis
        
        # Socket
        self.HOST = "localhost"
        self.PORT = 8888
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Struct object for the linescan
        self.linescan_struct = struct.Struct(2048*'H')
        # Matplotlib plot setup
        self.fig = pylab.figure(1)
        self.ax = self.fig.add_subplot(111)
        self.ax.grid(True)
        self.ax.set_title('Linescan plotter')
        self.ax.axis([0, 2048, 1088, 0])
        self.range = pylab.arange(0, 2048, 1)
        self.line1, = self.ax.plot(2048, 1088)
        self.manager = pylab.get_current_fig_manager()
        # Mouse input
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        # Buttons
        # Save button
        self.save_button_ax = pylab.axes([0.8, 0.91, 0.1, 0.075])
        self.save_button = Button(self.save_button_ax, 'Save')
        self.save_button.on_clicked(self.save_scanlines)
        # Stop button
        self.stop_button_ax = pylab.axes([0.23, 0.91, 0.1, 0.075])
        self.stop_button = Button(self.stop_button_ax, 'Stop')
        self.stop_button.on_clicked(self.stop_scan)
        # Start button
        self.start_button_ax = pylab.axes([0.125, 0.91, 0.1, 0.075])
        self.start_button = Button(self.start_button_ax, 'Start')
        self.start_button.on_clicked(self.start_scan)
        # Timer thread
        self.timer = self.fig.canvas.new_timer(interval=20)
        self.timer.add_callback(self.callback, ())
        self.timer.start()
        # Scan variables
        self.scan_range = []
        self.scanlines = []
        self.scanline_times = []
        self.start_scan_time = None
        self.stop_scan_time = None
        self.scan_time = None
        self.scanning = False
        
        
        # Get transforms
        self.get_laser_to_turntable_transform()        
        
        # Start        
        pylab.show()
    
        
    def callback(self, arg):
        self.sock.sendto(" ", (self.HOST, self.PORT))
        received = self.sock.recv(4096)
        rec = np.array(self.linescan_struct.unpack(received), np.float64)
        rec /= 2**6  # COG 6bit mode
        ud = self.undistort_points(rec)
        pt = self.perspective_transform(ud)
        ud.resize((len(rec),2))
        
        # Split array columnwise
        xarr, yarr = np.hsplit(ud, 2)
#        self.line1.set_data(self.x_range, rec)  # Plot raw pixels
        self.line1.set_data(xarr, yarr)  # Plot undistorted pixels
        if self.scanning:
            t = time.time()
            self.scanline_times.append(t)
            self.scanlines.append(rec[self.scan_range[0]:self.scan_range[1]])
            print(t - self.start_scan_time)
        self.manager.canvas.draw()
        
    def on_click(self, event):
#        print(dir(event))
        if len(self.scan_range) < 2:
            if event.button == 2:
                self.scan_range.append(event.xdata)
                print('Added: {x}'.format(x=int(event.xdata)))
        if event.button == 3:
            if not self.scanning:
                print('Start scanning!')
                self.scanning = True  # Start scan
            else:
                print('Stop scanning!')
                self.scanning = False
                
                
    def save_scanlines(self, event):
        print('Save pointcloud')
        self.transform_scanlines()
        self.save_pointcloud()
        
    def stop_scan(self, event):
        if self.scanning:
            print('Stop scanning!')
            self.stop_scan_time = time.time()
            self.scan_time = self.stop_scan_time - self.start_scan_time
            print('Scan duration: {t} seconds'.format(t=self.scan_time))
            self.scanning = False
        
    def start_scan(self, event):
        if len(self.scan_range) == 2:
            print('Start scanning!')
            self.start_scan_time = time.time()
            self.scanning = True
        else:
            print('Set range!')
            
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
        return ud
            
    def perspective_transform(self, scanline):
        pt = cv2.perspectiveTransform(scanline, self.homography).reshape(-1,2)
#        print(pt[1024][0])  # x, z
        return pt
        
#        print(pt[1024][0])  # x, z
        
    def transform_scanlines(self):
        print('Transforming pointcloud')
        l2t = self.get_laser_to_turntable_transform()
        w = self.angspeed
        st = self.start_scan_time
        sl = self.scanlines
        slt = self.scanline_times
        
        self.pc = np.empty((len(sl)*len(sl[0]), 3))
        
        pci = 0
        for i in range(len(sl)):  
            udp = self.undistort_points(sl[i])
            # x,z in laser coordinates
            self.ptp = self.perspective_transform(udp) 
            angle = w * (slt[i] - st)
            rot_trf = m3d.Transform(m3d.Orientation.new_rot_z(-angle), m3d.Vector()) * l2t
            
            for p in self.ptp:
                # create m3d Vector of each point in scanline
                p = m3d.Vector(p[0], 0, p[1])
                # transform to turntable coordinates
                tp = rot_trf * p 

                
                self.pc[pci] = tp.data
                pci += 1
                
    def get_laser_to_turntable_transform(self):
        self.laser_to_camera_trf = m3d.Transform(np.hstack((self.tvec.reshape(3), 
                                          self.rvec.reshape(3))))
        
        tbl_zaxis = m3d.Vector(self.axis).normalized()
        tbl_center = m3d.Vector(self.center)
        tbl_xaxis = (m3d.Vector.e1 - tbl_zaxis.y * tbl_zaxis).normalized()
        self.table_to_camera_trf = m3d.Transform.new_from_xzp(tbl_xaxis, 
                                                              tbl_zaxis, 
                                                              tbl_center)
        
        return self.table_to_camera_trf.inverse() * self.laser_to_camera_trf
#        return self.table_to_camera_trf.inverse()
        
        
    def save_pointcloud(self, ply_name='pc.ply'):

        
        pc = self.pc
        
        fc = open(ply_name, 'wt')
        fc.write('ply\n')
        fc.write('format ascii 1.0\n')
        fc.write('comment : laser scanner\n')
        fc.write('element vertex %d\n' % len(pc))
        fc.write('property float x\n')
        fc.write('property float y\n')
        fc.write('property float z\n')
        fc.write('end_header\n')
        np.savetxt(fc, pc, fmt='%+6.4f')
        fc.close()
        print('Point cloud saved')
        
        
        
        
        
        

        
if __name__ == '__main__':
    cm = np.load('params/cm.npy')
    dc = np.load('params/dc.npy')
    h = np.load('params/homography.npy')
    rvec_laser = np.load('params/rvec_laser_calib_obj.npy')
    tvec_laser = np.load('params/tvec_laser_calib_obj.npy')
    angspeed = np.load('params/turntable_angspeed.npy')
    center = np.load('params/turntable_center.npy')
    axis = np.load('params/turntable_axis.npy')
    
    lp = UDPLinescanPlotter(cm, dc, 
                            h, rvec_laser, tvec_laser,
                            angspeed, center, axis)



