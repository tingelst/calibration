from aravis  import Camera
from camera_calibrator import OpenCVCalibrator

class AravisCapture():
    def __init__(self, name):
        self.cam = Camera(name)
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
        frame = self.cam.get_frame(wait=True)
        return True, frame

    def cleanup(self):
        self.cam.stop_acquisition()
        self.cam.shutdown()



if __name__ == '__main__':
    try:
        #capture = AravisCapture("Prosilica-02-2130A-06106")
        capture = AravisCapture("AT-Automation Technology GmbH-20805103")
        boards = []
        output_dir = datetime.datetime.now().strftime("%Y-%m-%d") + "_" + capture.cam.name
        boards.append(ChessboardInfo(8,6,0.0245))
        calibrator = OpenCVCalibration(capture, boards, output_dir=output_dir)
        while True:
            time.sleep(1)
            
    finally:
        capture.cleanup()
