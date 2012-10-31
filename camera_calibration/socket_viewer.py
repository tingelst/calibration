# -*- coding: utf-8 -*-
"""
Created on Wed Oct 31 10:23:57 2012

@author: Lars Tingelstad
"""

import socket
import struct
import numpy as np
import cv2



class ImageViewer(object):
    
    def __init__(self):
        print('Image viewer')
        
        # Socket
        self.HOST = "localhost"
        self.PORT = 8888
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.im_struct = struct.Struct(2048*'B')
        self.img = []
        
    def run(self):
        ''' Main loop '''
        while(True):
            count = 0
            img = []
            self.sock.sendto("i", (self.HOST, self.PORT))
#            recv = self.sock.recv(2048)
#            img.append(self.im_struct.unpack(recv))
            while(True):
                self.sock.sendto("h", (self.HOST, self.PORT))
                recv = self.sock.recv(2048)
                if recv[:8] == 'finished':
                    break
                else:
                    img.append(self.im_struct.unpack(recv))
                
                count = count + 1


            img = np.array(img, np.uint8)
            img = cv2.pyrDown(img, dstsize = (1024, 544))
            cv2.imshow('image', img)
            c = cv2.waitKey(30)
            if c == 27:
                break
            del img
            
            
                
if __name__ == '__main__':
    ImageViewer().run()
                
        
        
        