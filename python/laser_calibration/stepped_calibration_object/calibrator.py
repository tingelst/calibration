'''
Created on Dec 18, 2012

@author: lars
'''

import matplotlib.pyplot as plt
import numpy as np
import cv2


class State:
    PLANE1, PLANE2, PLANE3, LASERLINE = range(4)
    
    
    
class CalibrationConfigReader(object):
    
    def __init__(self, filename):
        self._file = open(filename)
        
    def parse_calibration_config(self):
        pass

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
#        cv2.createTrackbar('blur', self._win_name, 5, 20, 
#                           self._median_blur)
        cv2.createTrackbar('threshold', self._win_name, 100, 255, 
                           self._threshold)
        cv2.createTrackbar('window', self._win_name, 30, 50, 
                           self._nothing)
  
        ## on mouse OpenCV
        cv2.setMouseCallback(self._win_name, self._on_mouse)
        ## on mouse Matplotlib      
#        self._cid = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        
        ## Laser line
        self._laserline = laserline.astype(np.float64) / 2**6
        self._laserline_limits = []
        ## List of numpy arrays with the laserline in each plane
        self._laserline_in_planes = []
        ## Fullframe image
        self._original_image = fullframe
        self._displayed_image = fullframe.copy()
        
        
        x_range =  np.arange(0, 2048, 1)
        self._laserline_2d = np.vstack((x_range, self._laserline)).transpose()
    
            
        ## Calibration data ###       
        
        ## Measured plane heights
        self._plane_heights = [35.0564, 59.9405, 69.9731, 59.9552, 35.0308]
        
        ## object points
        self._objpts_row1 = [[10.1901, 10.0119], [60.1105, 10.0269]]
        self._objpts_row2 = [[10.1896, 60.0365], [60.1135, 60.0787]]
        self._objpts_row3 = [[10.2052, 95.0378], [60.1271, 95.0745]]
        self._objpts_row4 = [[10.2068, 124.9785], [60.1215, 125.0333]]
        self._objpts_row5 = [[10.1520, 159.9692], [60.1071, 160.0248]]
        self._objpts_row6 = [[10.1842, 209.9638], [60.0773, 210.0800]]
        
        ## 24 3d object points 
        self._objpts_row1_3d = [o + [self._plane_heights[0]] for o in self._objpts_row1]
        self._objpts_row2_3d = [o + [self._plane_heights[1]] for o in self._objpts_row2]
        self._objpts_row3_3d = [o + [self._plane_heights[2]] for o in self._objpts_row3]
        self._objpts_row4_3d = [o + [self._plane_heights[2]] for o in self._objpts_row4]
        self._objpts_row5_3d = [o + [self._plane_heights[3]] for o in self._objpts_row5]
        self._objpts_row6_3d = [o + [self._plane_heights[4]] for o in self._objpts_row6]
        
        
        ## The sequence must be equal to the input
        self._objpts_3d = np.concatenate((np.array(self._objpts_row1_3d, np.float32), 
                                          np.array(self._objpts_row6_3d, np.float32), 
                                          np.array(self._objpts_row2_3d, np.float32),
                                          np.array(self._objpts_row5_3d, np.float32),
                                          np.array(self._objpts_row3_3d, np.float32),
                                          np.array(self._objpts_row4_3d, np.float32)))
                                          
#        print(self._objpts_3d)
        
        ## image points
        self._imgpts_row16 = []
        self._imgpts_row25 = []
        self._imgpts_row34 = []
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
        self._median_blur_trackbar()
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


            if self._state is not State.LASERLINE:
                x1, x2, y1, y2 = int(x) - win_size, int(x) + win_size, \
                int(y) - win_size, int(y) + win_size
                cv2.rectangle(temp, (x1 - 1, y1 - 1), (x2 + 1, y2 + 1), 
                          (255, 255, 0))
                roi = temp[y1:y2, x1:x2]    
                center_subpix_roi = self._fit_ellipse(roi)
                center_subpix_fullframe = np.array([np.float32(x1) + center_subpix_roi[0], 
                                                    np.float32(y1) + center_subpix_roi[1]])
#            print(center_subpix_fullframe)

            
            if self._state == State.PLANE1:
                if len(self._imgpts_row16) < 4:  ## 4
                    self._imgpts_row16.append(center_subpix_fullframe)
                    cv2.putText(temp, '{0}'.format(len(self._imgpts_row16)), 
                                (x1,y1+win_size), cv2.FONT_HERSHEY_SIMPLEX,
                                1, (255,0,255))

                else:
                    print('got all points in plane 1 and 5')
            elif self._state == State.PLANE2:
                if len(self._imgpts_row25) < 4:  ## 4
                    self._imgpts_row25.append(center_subpix_fullframe)
                    cv2.putText(temp, '{0}'.format(len(self._imgpts_row25)), 
                                (x1,y1+win_size), cv2.FONT_HERSHEY_SIMPLEX,
                                1, (255,0,255))
                else:
                    print('got all points in plane 2 and 4')
            elif self._state == State.PLANE3:
                if len(self._imgpts_row34) < 4:  ## 4  
                    self._imgpts_row34.append(center_subpix_fullframe)
                    cv2.putText(temp, '{0}'.format(len(self._imgpts_row34)), 
                                (x1,y1+win_size), cv2.FONT_HERSHEY_SIMPLEX, 
                                1, (255,0,255))
                else:
                    print('got all points in plane 3')
            elif self._state == State.LASERLINE:
                if len(self._laserline_limits) < 10:
                    self._laserline_limits.append(x)

            
            self._displayed_image = temp
            self._update()
        
        elif event == cv2.EVENT_RBUTTONDOWN:
            ## find plane homography
            if self._state == State.PLANE1:
                ## undistort and nom
                pts = self._undistort_points(np.array(self._imgpts_row16, np.float32))[1]
                self._homography_plane1, mask = cv2.findHomography(pts, np.array(self._objpts_row1 + self._objpts_row6, np.float32))
                self._homography_matrices.append(self._homography_plane1)
            elif self._state == State.PLANE2:
                pts = self._undistort_points(np.array(self._imgpts_row25, np.float32))[1]
                self._homography_plane2, mask = cv2.findHomography(pts, np.array(self._objpts_row2 + self._objpts_row5, np.float32))
                self._homography_matrices.append(self._homography_plane2)    
            elif self._state == State.PLANE3:
                pts = self._undistort_points(np.array(self._imgpts_row34, np.float32))[1]
                self._homography_plane3, mask = cv2.findHomography(pts, np.array(self._objpts_row3 + self._objpts_row4, np.float32))
                self._homography_matrices.append(self._homography_plane3)
                ## find object pose
                self._imgpts = np.concatenate((np.array(self._imgpts_row16, np.float32), 
                                             np.array(self._imgpts_row25, np.float32),
                                             np.array(self._imgpts_row34, np.float32)))
                                             
                self._homography_matrices.append(self._homography_plane2)
                self._homography_matrices.append(self._homography_plane1)

                                             
                                                                                          
                                             
                ## Not necessary to undistort the points before solvePnP
                ret, rvec, self._tvec = cv2.solvePnP(self._objpts_3d, self._imgpts, self._cm, self._dc)
                self._rmat, jac = cv2.Rodrigues(rvec)
                print('Rotation matrix:')
                print(self._rmat)
                print('Translation vector:')
                print(self._tvec)
                
                self._draw_laserline()
                print('Select the limits for the laserline on the calibration object - from left to right')

            elif self._state == State.LASERLINE:
                
                ## slice laserline in planes
                self._laserline_in_planes.append(self._laserline_2d[self._laserline_limits[0]:self._laserline_limits[1]])
                self._laserline_in_planes.append(self._laserline_2d[self._laserline_limits[2]:self._laserline_limits[3]])
                self._laserline_in_planes.append(self._laserline_2d[self._laserline_limits[4]:self._laserline_limits[5]])
                self._laserline_in_planes.append(self._laserline_2d[self._laserline_limits[6]:self._laserline_limits[7]])
                self._laserline_in_planes.append(self._laserline_2d[self._laserline_limits[8]:self._laserline_limits[9]])
                undistorted_laserline = []
                [undistorted_laserline.append(self._undistort_points(l)[0]) for l in self._laserline_in_planes]
                perspective_transformed_laserline = []
                [perspective_transformed_laserline.append(self._perspective_transform(l, h)) for l, h in zip(undistorted_laserline, self._homography_matrices)]
                ptp_laser = []
                for h, l in zip(self._plane_heights, perspective_transformed_laserline):
                        for i in range(len(l)):
                            ptp_laser.append(np.append(l[i], h))
                            
                self._ptp_laserline_3d_in_object = np.array(ptp_laser, dtype=np.float32)
#                print('Points in object')
#                print(self._ptp_laserline_3d_in_object)

                self._get_laserline_object_coords_in_camera(self._ptp_laserline_3d_in_object)
                print(self._ptp_laserline_3d_in_camera)
                print('laser_plane equation in camera coordinate system')
                self._fit_plane_to_laserline_object_coords(self._ptp_laserline_3d_in_camera)
                print(self._laserplane_eq)
                

              
                
#                self._ud = np.concatenate((undistorted_laserline[0],
#                                     undistorted_laserline[1],
#                                     undistorted_laserline[2],
#                                     undistorted_laserline[3],
#                                     undistorted_laserline[4]))
#                self._ud.resize(self._ud.shape[0],1,2)
#                print(self._ud.shape)
#                
#                self._camera_laser_plane_homography, mask = cv2.findHomography(self._ptp_laserline_3d_in_camera.resize(self._ptp_laserline_3d_in_camera.shape[0], 1, 2).astype(np.float32), self._ud.astype(np.float32))
#                print(self._camera_laser_plane_homography)

                
#                print(ud.shape)
#                for i in range(1, len(undistorted_laserline)):
#                    print(undistorted_laserline[i].shape)
#                    ud = np.vstack((ud, undistorted_laserline[i])).resize(ud.shape[0], 2)
#                print(ud.shape)
#                print(np.array(undistorted_laserline, np.float32))
                
                

                
                        
#                print(perspective_transformed_laserline)
#                print(undistorted_laserline)
#                for l in self._laserline_in_planes:
#                    udp = self._undistort_points(l)
#                    undistorted_laserline.append()
                
                ## get xyz for all laserline points using the homography matrices
                ## and the calibrated plane heights

                                
                
            
            ## change _state
            if self._state is not State.LASERLINE:
                self._state += 1
#            print(self._homography_matrices)
        
        elif event == cv2.EVENT_MBUTTONDOWN:
            # # Restart calibration procedure
            self._displayed_image = self._original_image.copy()
            self._threshold()
            self._update()
            
    def _draw_laserline(self):
        img = self._displayed_image.copy()
        for i in  range(len(self._laserline)):
            cv2.circle(img, (i, int(self._laserline[i])), 2, (0, 0, 255), -1)
        self._displayed_image = img
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
        ud_resize = ud.copy()
        ud_resize.resize((length,2))
        return ud, ud_resize
        
        
    def _perspective_transform(self, laserline, homography):
        pt = cv2.perspectiveTransform(laserline.astype(np.float64), homography).reshape(-1,2)
        return pt
    
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
            ## Find the outer
            if len(contour) > 40:
                ## Fit ellipse to contour
                ellipse = cv2.fitEllipse(contour)
                ## Draw ellipse
                cv2.ellipse(roi, ellipse, color=(0, 0, 255))
        center = ellipse[0]
        ## Draw center
        cv2.circle(roi, (int(center[0]), int(center[1])), 2, (0, 255, 0), -1)
        
        temp = cv2.resize(roi, dsize=(roi.shape[0] * 10, roi.shape[1] * 10))
        
        cv2.imshow('roi', temp)
        return center
    
        
    def _threshold(self, *arg, **kw):
        ''' 
        Threshold the image using the value from the threshold 
        trackbar and update the image
        '''
        
        org = self._original_image.copy()
        thrs = cv2.getTrackbarPos('threshold', self._win_name)
        blurred = self._median_blur(org)
        ret, image = cv2.threshold(blurred, 
                                   thrs, 255, cv2.THRESH_BINARY)
        self._displayed_image = image
        self._update()
        
    def _median_blur(self, roi):
        kernel_size = 11
        blur = cv2.bilateralFilter(roi,kernel_size, kernel_size*2,kernel_size/2)
#        blur = cv2.blur(roi, (11,11))
        return blur
        
    def _median_blur_trackbar(self, *arg, **kw):
        blur_val = cv2.getTrackbarPos('blur', self._win_name)
        median_blur = cv2.medianBlur(self._displayed_image.copy(),
                                     blur_val)
        self._displayed_image = median_blur
        self._update(median_blur)

        
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
        
    
    def _fit_plane_to_laserline_object_coords(self, coords):
        '''
        Fit a plane the laserline object coords 
        '''
        x, y, z = np.hsplit(coords, 3)
        x0 = np.mean(x)
        y0 = np.mean(y)
        z0 = np.mean(z)
        print('mean')
        mean = np.array([x0, y0, z0])
        print(mean)
        u,s,vh = np.linalg.linalg.svd(coords - mean)
        v = vh.conj().transpose()
        print(v)
        self._laserplane_eq = v[:,-1] 
        
        
    def _get_laserline_object_coords_in_camera(self, coords):
        ptp_laserline_3d_in_camera = []
        for p_in_object in coords:
            p_in_camera = self._tvec + np.dot(self._rmat, p_in_object.reshape(3,1))
            ptp_laserline_3d_in_camera.append(p_in_camera.reshape(3))
        self._ptp_laserline_3d_in_camera = np.array(ptp_laserline_3d_in_camera, dtype=np.float32)
            
        


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
#    img = cv2.imread('justleds.tif', 0)
    fullframe = np.load('fullframe_test5_40000.npy')
    laserline = np.load('laserline_test5_250.npy')
#    img = cv2.bitwise_not(img)
    
    lc = Calibrator(fullframe, laserline, camera_matrix=cm, dist_coeffs=dc)        
        
        
        
