import aravis 
from camera_calibrator import OpenCVCalibrator

class AravisCapture():
    def __init__(self):
        ar = aravis.Aravis()
        #self.cam = ar.get_camera("Prosilica-02-2130A-06106")
        self.cam = ar.get_camera("AT-Automation Technology GmbH-20805103")
        x, y, width, height = self.cam.get_region()
        print("Camera model: ", self.cam.get_model_name())
        print("Vendor Name: ", self.cam.get_vendor_name())
        print("Device id: ", self.cam.get_device_id())
        print("Image size: ", width, ",", height)
        sensor =  self.cam.get_sensor_size() 
        print("Sensor size: ", self.cam.get_sensor_size()) 
        print("Frame rate: ", self.cam.get_frame_rate())
        print("PacketSize: ", self.cam.get_integer_feature("GevSCPSPacketSize"))
        self.cam.set_integer_feature("AoiHeight", 2048)
        self.cam.set_frame_rate(20)
        self.cam.set_string_feature("CameraMode", "Image")
        self.cam.set_string_feature("PixelFormat", "Mono8")
        self.cam.set_integer_feature("GevSCPSPacketSize", 1500)
        self.cam.setup_stream(10)
        self.cam.start_acquisition_continuous()
        print "camera started"
        self.counter = 0

    def read(self):
        self.counter += 1
        print "trying to get frame: ", self.counter
        frame = None
        while frame == None:
            frame = self.cam.try_get_frame()
            time.sleep(0.001)
        print frame
        return True, frame
        #return True, cv.fromarray(frame)

    def cleanup(self):
        self.cam.stop_acquisition()
        self.cam.cleanup()



if __name__ == '__main__':
    try:
        capture = AravisCapture()
        boards = []
        output_dir = datetime.datetime.now().strftime("%Y-%m-%d") + "_" + capture.cam.name
        boards.append(ChessboardInfo(8,6,0.0245))
        calibrator = OpenCVCalibration(capture, boards, output_dir=output_dir)
        while True:
            time.sleep(1)
            
    finally:
        capture.cleanup()
