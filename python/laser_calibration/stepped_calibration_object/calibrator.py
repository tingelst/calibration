'''
Created on Dec 18, 2012

@author: lars
'''


import numpy as np
import cv2
import cv3d

class State:
    PLANE1, PLANE2, PLANE3 = range(3)

class Calibrator(object):
    '''
    Laser calibrator class
    '''

    def __init__(self, fullframe, laserline=None, 
                 camera_matrix=None, dist_coeffs=None):
        
        ## camera calibration matrices
        self._cm = camera_matrix
        self._dc = dist_coeffs    
        
        ## OpenCV setup 
        self._win_name = 'Laser calibrator - Lars Tingelstad, IPK, NTNU'
        self._window = cv2.namedWindow(self._win_name)
        ## trackbars
        cv2.createTrackbar('threshold', self._win_name, 100, 255, 
                           self._threshold)
        cv2.createTrackbar('window', self._win_name, 16, 50, 
                           self._nothing)
        ##on mouse
        cv2.setMouseCallback(self._win_name, self._on_mouse)
        
        ## Laser line
        self._laserline = laserline
        ## Fullframe image
        self._original_image = fullframe
        self._displayed_image = fullframe.copy()
        
        
        ## Measured plane heights
        self._plane_heights = [0.0, 25.0, 40.0]
        
        ## object points
        self._objpts_plane1 = [[-100, 10], [-100, 60], [-100, 110], [-100, 190],
                               [100, 190], [100, 110], [100, 60], [100, 10]]
        self._objpts_plane2 = [[-50, 10], [-50, 60], [-50, 110], [-50, 190],
                               [50, 190], [50, 110], [50, 60], [50, 10]]     
        self._objpts_plane3 = [[-15, 10], [-15, 60], [-15, 110], [-15, 190],
                               [15, 190], [15, 110], [15, 60], [15, 10]]
        
        ## 24 3d object points 
        objpts_plane1_3d = np.array([o + [self._plane_heights[0]] 
                                     for o in self._objpts_plane1], np.float32)
        objpts_plane2_3d = np.array([o + [self._plane_heights[1]] 
                                     for o in self._objpts_plane2], np.float32)
        objpts_plane3_3d = np.array([o + [self._plane_heights[2]] 
                                     for o in self._objpts_plane3], np.float32)
        self._objpts_3d = np.concatenate((objpts_plane1_3d, 
                                          objpts_plane2_3d, 
                                          objpts_plane3_3d))
        
        ## image points
        self._imgpts_plane1 = []
        self._imgpts_plane2 = []
        self._imgpts_plane3 = []
        self._imgpts_undistorted = None
        
        ### RESULTS ###
        ## calculated homography matrices
        self._homography_matrices = []
        ## calibration object pose
        self._obj_pose = None
        
        ## initial _state        
        self._state = State.PLANE1
        ## Start 
        self._threshold()
        cv2.waitKey()
        
    def _on_mouse(self, event, x, y, flags, param):
        x, y = np.int16([x, y])
        if event == cv2.EVENT_LBUTTONDOWN:
            ## click and fit ellipse
            temp = self._displayed_image.copy()
            ## convert image to greyscale
            if len(self._displayed_image.shape) == 2:
                temp = cv2.cvtColor(temp, cv2.COLOR_GRAY2BGR)
            ## find center of ellipse within the specified ROI
            win_size = cv2.getTrackbarPos('window', self._win_name)
            x1, x2, y1, y2 = int(x) - win_size, int(x) + win_size, \
                             int(y) - win_size, int(y) + win_size
            cv2.rectangle(temp, (x1 - 1, y1 - 1), (x2 + 1, y2 + 1), 
                          (255, 255, 0))

            roi = temp[y1:y2, x1:x2]
            center_subpix_roi = self._fit_ellipse(roi)
            center_subpix_fullframe = np.array([np.float32(x1) + center_subpix_roi[0], 
                                                np.float32(y1) + center_subpix_roi[1]])

            
            if self._state == State.PLANE1:
                if len(self._imgpts_plane1) < 8:  ## 8
                    self._imgpts_plane1.append(center_subpix_fullframe)
                    cv2.putText(temp, '{0}'.format(len(self._imgpts_plane1)), 
                                (x1,y1+win_size), cv2.FONT_HERSHEY_SIMPLEX,
                                1, (255,0,255))
                else:
                    print('got all points in plane 1')
            elif self._state == State.PLANE2:
                if len(self._imgpts_plane2) < 8:  ## 8
                    self._imgpts_plane2.append(center_subpix_fullframe)
                    cv2.putText(temp, '{0}'.format(len(self._imgpts_plane2)), 
                                (x1,y1+win_size), cv2.FONT_HERSHEY_SIMPLEX,
                                1, (255,0,255))
                else:
                    print('got all points in plane 2')
            elif self._state == State.PLANE3:
                if len(self._imgpts_plane3) < 8:  ## 8  
                    self._imgpts_plane3.append(center_subpix_fullframe)
                    cv2.putText(temp, '{0}'.format(len(self._imgpts_plane3)), 
                                (x1,y1+win_size), cv2.FONT_HERSHEY_SIMPLEX, 
                                1, (255,0,255))
                else:
                    print('got all points in plane 3')
            
            self._displayed_image = temp
            self._update()
        
        elif event == cv2.EVENT_RBUTTONDOWN:
            ## find plane homography
            if self._state == State.PLANE1:
                ## undistort and nom
                pts = self._undistort_points(np.array(self._imgpts_plane1, np.float32))
#                print pts
                homography, mask = cv2.findHomography(pts, np.array(self._objpts_plane1, np.float32))
                self._homography_matrices.append(homography)
            elif self._state == State.PLANE2:
                pts = self._undistort_points(np.array(self._imgpts_plane2, np.float32))
                homography, mask = cv2.findHomography(pts, np.array(self._objpts_plane2, np.float32))
                self._homography_matrices.append(homography)    
            elif self._state == State.PLANE3:
                pts = self._undistort_points(np.array(self._imgpts_plane3, np.float32))
                homography, mask = cv2.findHomography(pts, np.array(self._objpts_plane3, np.float32))
                self._homography_matrices.append(homography)
                ## find object pose
                self._imgpts = np.concatenate((np.array(self._imgpts_plane1, np.float32), 
                                             np.array(self._imgpts_plane2, np.float32),
                                             np.array(self._imgpts_plane3, np.float32)))
                ## Not necessary to undistort the points before solvePnP
                ret, rvec, tvec = cv2.solvePnP(self._objpts_3d, self._imgpts, self._cm, self._dc)
                rmat, jac = cv2.Rodrigues(rvec)
                print rvec
                print tvec
            
            ## change _state
            if self._state is not State.PLANE3:
                self._state += 1
                
            
            print(self._homography_matrices)
 
        
        elif event == cv2.EVENT_MBUTTONDOWN:
            # # Restart calibration procedure
            self._displayed_image = self._original_image.copy()
            self._threshold()
            self._update()    
            
            
    def _undistort_points(self, points):
        ''' 
        Wrapper method for the opencv undistortPoints method. 
        Returns undistorted and normalized points using the
        calibration matrices.
        '''
        length = len(points)
        # Create temp array
        temp = np.zeros((length,1,2), dtype=np.float32)
        # Copy scanline into temp array
        for i in range(length):
            temp[i][0][0] = points[i][0]
            temp[i][0][1] = points[i][1]
        # Undistort and normalize
        ud = cv2.undistortPoints(temp, self._cm, self._dc)
        # Resize array to shape (-1, 2)
        ud.resize((length,2))
        return ud 
    
    def _fit_ellipse(self, roi):
        ''' 
        Extract contour(s) from the region of interest and fit an ellipse 
        to the contour. Returns the center of the ellipse in the region
        of interest with subpixel accuracy
        '''
        
        temp = roi.copy()
        if len(temp.shape) == 3:
            temp = cv2.cvtColor(temp, cv2.COLOR_BGR2GRAY)
        contours, hierarchy = cv2.findContours(temp, cv2.RETR_LIST, 
                                               cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if len(contour) > 5:
                ellipse = cv2.fitEllipse(contour)
                cv2.ellipse(roi, ellipse, color=(0, 0, 255))
        center = ellipse[0]
        cv2.circle(roi, (int(center[0]), int(center[1])), 2, (0, 255, 0), -1)
        temp = cv2.resize(roi, dsize=(roi.shape[0] * 4, roi.shape[1] * 4))
        cv2.imshow('roi', temp)
        return center
        
    def _threshold(self, *arg, **kw):
        ''' 
        Threshold the image using the value from the threshold 
        trackbar and update the image
        '''
        thrs = cv2.getTrackbarPos('threshold', self._win_name)
        ret, image = cv2.threshold(self._original_image.copy(), 
                                   thrs, 255, cv2.THRESH_BINARY)
        self._displayed_image = image
        self._update(image)
        
    def _extract_laserline_in_planes(self):
        '''
        Extract the points in the laserline that lies in each plane
        on the calibration object
        '''
        pass

    def _get_laserline_object_coords(self):
        '''
        Calculate object coordinates for the laserline using the calculated
        homography matrices and the measured plane heights 
        '''
        pass
    
    def fit_plane_to_laserline_object_coords(self):
        '''
        Fit a plane the laserline object coords 
        '''

        
    def _calibrate(self):
        ''' 
        Calculate image plane to laserplane homography as well as
        the transformation from the laser plane coordinate system to
        the object (world) coordinate system
        '''
        pass
              
        
    def _update(self, *arg, **kw):
        ''' 
        Display the updated image
        '''
#        cv2.imshow('Original image', self._original_image)
        cv2.imshow(self._win_name, self._displayed_image)
        self._wait_key()
            
    def _wait_key(self):
        c = cv2.waitKey(30)
        if c == 27:
            cv2.destroyAllWindows()
            
    def _nothing(self, *arg, **kw):
        """Convenience method"""
        pass
           
        
        


        
        
        
        
        
        
        
        
        
if __name__ == '__main__':
    
    cm = np.load('cm.npy')
    dc  = np.load('dc.npy')
    img = cv2.imread('snapshot1.png', 0)
#    img = cv2.bitwise_not(img)
    
    lc = Calibrator(img, None, cm, dc)        
        
        
        
