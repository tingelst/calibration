# -*- coding: utf-8 -*-
"""
Created on Wed Oct 31 10:23:57 2012

@author: Lars Tingelstad
"""

import socket
import struct
import numpy as np
import cv2
import time


class ImageViewer(object):
    
    def __init__(self):
        print('Image viewer')
        
        # Socket
        self.host = 'localhost'
        self.port = 8888
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        
        self.sock.connect((self.host, self.port))
        self.im_struct = struct.Struct(2048*1088*'B')
        self.img = []
        
    def run(self):
        ''' Main loop '''
        
        t = time.time()
        count = 0

        while(True):

            img = []
            recv = ""
            self.sock.send("s")  # s = start
            t = time.time()
            while(len(recv) < 2048*1088):
                recv += self.sock.recv(8192)
                
            frame = np.frombuffer(recv, np.uint8)
            frame.resize(1088, 2048)
            frame = cv2.pyrDown(frame, dstsize = (1024, 544))
            cv2.imshow('image', frame)
            c = cv2.waitKey(30)
            if c == 27:
                break
            
            t = time.time() - t
            print(1.0 / t)

            
                
#            img = np.array(img, np.uint8)
##            np.save('image_{count}.npy'.format(count=count), img)


            
            del recv
            del img
            
            
                
if __name__ == '__main__':
    iv = ImageViewer()
    iv.run()
                
        
        
        