class GigECapture(threading.Thread):
    def __init__(self):
        self.cam = None
        self.pv = pvlib.PvLib()
        cams = self.pv.getCameras()
        if not cams:
            print('Error getting camera list')
        else:
            self.cam = cams[0]
            self.cam.startCaptureTrigger()

    def read(self):
        frame = self.cam.getNumpyArray()
        return True, frame


if __name__ == '__main__':
    capture = GigECapture()
    boards = []
    boards.append(ChessboardInfo(8,6,0.0245))
    calibrator = OpenCVCalibration(capture, boards)
    while True:
        time.sleep(1)
 



