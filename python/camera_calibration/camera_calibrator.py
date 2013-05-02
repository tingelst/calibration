import sys
import time

import cv2
import cv2.cv as cv

import os
import threading
import Queue

from camera_calibrator_base import MonoCalibrator, ChessboardInfo
# BUG!
ESC = 536870939

class GrabberThread(threading.Thread):
    def __init__(self, capture, queue):
        threading.Thread.__init__(self)
        self.capture = capture
        self.queue = queue
        print(self.__class__)

    def run(self):
        while True:
            ret, img = self.capture.read()
            self.queue(img)

class ConsumerThread(threading.Thread):
    def __init__(self, queue, function):
        threading.Thread.__init__(self)
        self.queue = queue
        self.function = function
        print(self.__class__)

    def run(self):
        while True:
            while True:
                m = self.queue.get()
                if self.queue.empty():
                    break
            self.function(m)

class CameraCalibration(object):

    def __init__(self, capture, boards, flags=0):

        self._boards = boards
        self._calib_flags = flags

        ## Initialise image queue
        self.q_mono = Queue.Queue()

        ## Initialise image grabber
        grabber = GrabberThread(capture, self.queue_monocular)
        grabber.setDaemon(True)
        grabber.start()

        mth = ConsumerThread(self.q_mono, self.handle_monocular)
        mth.setDaemon(True)
        mth.start()

        self.calibrator = MonoCalibrator(self._boards, self._calib_flags)

        print(self.__class__)

    def redraw_monocular(self, *args):
        pass

    def queue_monocular(self, img):
        self.q_mono.put(img)

    def handle_monocular(self, img):
        drawable = self.calibrator.handle_msg(img)
        self.displaywidth = drawable.scrib.cols
        self.redraw_monocular(drawable)

    def do_upload(self):
        print('do_upload')

class OpenCVCalibration(CameraCalibration):
    """ Calibration node with an OpenCV Gui """

    def __init__(self, capture, boards, output_dir="./", flags=0):

        CameraCalibration.__init__(self, capture=capture, boards=boards, flags=flags)
        self.window_name = 'Camera calibration'
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.2, 1, thickness=1)
        self.show_undistorted = False
        self.saved_calibration = False
        self.output_dir = output_dir

    def waitkey(self):
        k = cv.WaitKey(6)
        if k in [536870939, 27, ord('q')]:
            sys.exit()
        elif k in [536871011, ord('c'), 1048675]:
            if self.calibrator.goodenough:
                print('Calibrating camera')
                self.calibrator.do_calibration()
                self.save_calib()
            else:
                print('Not enough good samples')
        elif k in [536871027, ord('s'), 1048691]:
            if self.calibrator.calibrated and not self.saved_calibration:
                self.calibrator.do_save()
                self.saved_calibration = True
            else:
                print('Camera is not calibrated. Cannot save calibration')
        elif k in [536871029, ord('u')]:
            if not self.show_undistorted:
                self.show_undistorted = True
            else:
                self.show_undistorted = False
        #print k
        return k

    def save_calib(self):
        print('Saving calibration to ', self.output_dir)
        numpy.save(os.path.join(self.output_dir, 'dc.npy'), numpy.asarray(self.calibrator.distortion, numpy.float32))
        numpy.save(os.path.join(self.output_dir, 'cm.npy'), numpy.asarray(self.calibrator.intrinsics, numpy.float32))
        cv.Save(os.path.join(self.output_dir, 'camera_matrix.xml'), self.calibrator.intrinsics)
        cv.Save(os.path.join(self.output_dir, 'distortion_coefficients.xml'), self.calibrator.distortion)

    def y(self, i):
        """Set up right-size images"""
        return 30 + 40 * i

    def screendump(self, im):
        i = 0
        while os.access("dump%d.png" % i, os.R_OK):
            i += 1
        cv.SaveImage("dump%d.png" % i, im)

    def redraw_monocular(self, drawable):
        width, height = cv.GetSize(drawable.scrib)

        display = cv.CreateMat(max(480, height), width, cv.CV_8UC3)
        cv.Zero(display)
        cv.Copy(drawable.scrib, cv.GetSubRect(display, (0,0,width,height)))

        if not self.calibrator.calibrated:
            if drawable.params:
                 for i, (label, lo, hi, progress) in enumerate(drawable.params):
                    (w,_),_ = cv.GetTextSize(label, self.font)
                    cv.PutText(display, label, (width + (-w - 100) / 2, self.y(i)), self.font, (255,255,255))
                    color = (0,255,0)
                    if progress < 1.0:
                        color = (0, int(progress*255.), 255)
                    cv.Line(display,
                            (int(width -100 + lo * 100), self.y(i) + 20),
                            (int(width -100  + hi * 100), self.y(i) + 20),
                            color, 4)
                    if self.calibrator.goodenough:     
                        text_cal = 'Press \'c\' to calibrate camera and save calibration'
                        (w,_),_ = cv.GetTextSize(text_cal, self.font)
                        cv.PutText(display, text_cal, 
                               (width - w - 10, height - 10), self.font, (0,255,0))
   
                    else:     
                        text = 'Show calibration pattern to camera'
                        (w,_),_ = cv.GetTextSize(text, self.font)
                        cv.PutText(display, text, 
                               (width - w - 10, height - 10), self.font, (255,255,255))  
                
            elif not drawable.params:
                text = 'Show calibration pattern to camera'
                (w, h),_ = cv.GetTextSize(text, self.font)
                cv.PutText(display, text,
                           (width - w - 10, height - 10), self.font, (255,255,255))

        else:
            if self.show_undistorted:
                display = self.undistort_image(display)
            text_calibrated = 'Calibrated'
            (w, h),_ = cv.GetTextSize(text_calibrated, self.font)
            cv.PutText(display, text_calibrated, 
                       (10, h + 10), self.font, (0,255,0)) 
            if not self.saved_calibration:
                text_save = 'Press \'s\' to save calibration to current dir'
                (w, h),_ = cv.GetTextSize(text_save, self.font)
                cv.PutText(display, text_save, 
                       (width - w - 10, height - 10), self.font, (0,255,0))
            
#            cv.PutText(display, "lin.", (width, self.y(0)), self.font, (0,0,0))
#            linerror = drawable.linear_error
#            if linerror < 0:
#                msg = "?"
#            else:
#                msg = "%.2f" % linerror
#                #print "linear", linerror
#            cv.PutText(display, msg, (width, self.y(1)), self.font, (0,0,0))

        self.show(display)
        
    def undistort_image(self, im):
        cm = cv.Load('camera_matrix.xml')
        dc = cv.Load('distortion_coefficients.xml')
        width, height = cv.GetSize(im)
        display = cv.CreateMat(height, width, cv.CV_8UC3)
        cv.Undistort2(im, display, cm, dc)
        return display

    def show(self, im):
        cv.ShowImage(self.window_name, im)
        k = self.waitkey()

#class ATC4Capture(threading.Thread):
#    def __init__(self):
#
#        # Socket
#        self.host = "169.254.208.224"
#        self.port = 9999
#        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
#        self.sock.connect((self.host, self.port))
#
#    def read(self):
#        recv = ""
#        self.sock.send("s")  # s = start
#        while(len(recv) < 2048*1088):
#            recv += self.sock.recv(8192)
#        frame = np.frombuffer(recv, np.uint8)
#        frame.resize(1088, 2048)
#        return True, frame

#class GigECapture(threading.Thread):
#    def __init__(self):
#        self.cam = None
#        self.pv = pvlib.PvLib()
#        cams = self.pv.getCameras()
#        if not cams:
#            print('Error getting camera list')
#        else:
#            self.cam = cams[0]
#            self.cam.startCaptureTrigger()
#
#    def read(self):
#        frame = self.cam.getNumpyArray()
#        return True, frame



if __name__ == '__main__':
    capture = cv2.VideoCapture(0)
    #    capture = GigECapture()
    #capture = ATC4Capture()
    capture = AravisCapture()
    boards = []
    boards.append(ChessboardInfo(8,6,0.0245))
    calibrator = OpenCVCalibration(capture, boards)
    while True:
        time.sleep(1)
            
