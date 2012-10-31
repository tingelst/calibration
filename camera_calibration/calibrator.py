import cv2
import cv2.cv as cv
import numpy as np

import os

import threading
import Queue

from ros_calibrator import MonoCalibrator, ChessboardInfo
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

class OpenCVCalibration(CameraCalibration):
    """ Calibration node with an OpenCV Gui """

    def __init__(self, *args):

        CameraCalibration.__init__(self, *args)
        cv.NamedWindow("display")
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.20, 1, thickness = 2)
        cv.SetMouseCallback("display", self.on_mouse)
        cv.CreateTrackbar("scale", "display", 0, 100, self.on_scale)

    def on_mouse(self, event, x, y, flags, param):
        if event == cv.CV_EVENT_LBUTTONDOWN and self.displaywidth < x:
            if self.calibrator.goodenough:
                if 180 <= y < 280:
                    self.c.do_calibration()
            if self.calibrator.calibrated:
                if 280 <= y < 380:
                    self.calibrator.do_save()
                elif 380 <= y < 480:
                    # Only shut down if we set camera info correctly, #3993
                    if self.do_upload():
                        print('Should quit')

    def waitkey(self):
        k = cv.WaitKey(6)
        if k in [27, ord('q')]:
            rospy.signal_shutdown('Quit')
        return k

    def on_scale(self, scalevalue):
        if self.calibrator.calibrated:
            self.calibrator.set_alpha(scalevalue / 100.0)

    def button(self, dst, label, enable):
        cv.Set(dst, (255, 255, 255))
        size = cv.GetSize(dst)
        if enable:
            color = cv.RGB(155, 155, 80)
        else:
            color = cv.RGB(224, 224, 224)
        cv.Circle(dst, (size[0] / 2, size[1] / 2), min(size) / 2, color, -1)
        ((w, h), _) = cv.GetTextSize(label, self.font)
        cv.PutText(dst, label, ((size[0] - w) / 2, (size[1] + h) / 2), self.font, (255,255,255))

    def buttons(self, display):
        x = self.displaywidth
        self.button(cv.GetSubRect(display, (x,180,100,100)), "CALIBRATE", self.calibrator.goodenough)
        self.button(cv.GetSubRect(display, (x,280,100,100)), "SAVE", self.calibrator.calibrated)
        self.button(cv.GetSubRect(display, (x,380,100,100)), "COMMIT", self.calibrator.calibrated)

    def y(self, i):
        """Set up right-size images"""
        return 30 + 40 * i

    def screendump(self, im):
        i = 0
        while os.access("/tmp/dump%d.png" % i, os.R_OK):
            i += 1
        cv.SaveImage("/tmp/dump%d.png" % i, im)

    def redraw_monocular(self, drawable):
        width, height = cv.GetSize(drawable.scrib)

        display = cv.CreateMat(max(480, height), width + 100, cv.CV_8UC3)
        cv.Zero(display)
        cv.Copy(drawable.scrib, cv.GetSubRect(display, (0,0,width,height)))
        cv.Set(cv.GetSubRect(display, (width,0,100,height)), (255, 255, 255))

        self.buttons(display)
        if not self.calibrator.calibrated:
            if drawable.params:
                 for i, (label, lo, hi, progress) in enumerate(drawable.params):
                    (w,_),_ = cv.GetTextSize(label, self.font)
                    cv.PutText(display, label, (width + (100 - w) / 2, self.y(i)), self.font, (0,0,0))
                    color = (0,255,0)
                    if progress < 1.0:
                        color = (0, int(progress*255.), 255)
                    cv.Line(display,
                            (int(width + lo * 100), self.y(i) + 20),
                            (int(width + hi * 100), self.y(i) + 20),
                            color, 4)

        else:
            cv.PutText(display, "lin.", (width, self.y(0)), self.font, (0,0,0))
            linerror = drawable.linear_error
            if linerror < 0:
                msg = "?"
            else:
                msg = "%.2f" % linerror
                #print "linear", linerror
            cv.PutText(display, msg, (width, self.y(1)), self.font, (0,0,0))

        self.show(display)


    def show(self, im):
        cv.ShowImage("display", im)
        if self.waitkey() == ord('s'):
            self.screendump(im)



if __name__ == '__main__':
    capture = cv2.VideoCapture(0)
    boards = []
    boards.append(ChessboardInfo(8,6,0.035))
    calibrator = OpenCVCalibration(capture, boards)
