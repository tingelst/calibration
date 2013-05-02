
class ATC4Capture(threading.Thread):
    def __init__(self):

        # Socket
        self.host = "169.254.208.224"
        self.port = 9999
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.connect((self.host, self.port))

    def read(self):
        recv = ""
        self.sock.send("s")  # s = start
        while(len(recv) < 2048*1088):
            recv += self.sock.recv(8192)
        frame = np.frombuffer(recv, np.uint8)
        frame.resize(1088, 2048)
        return True, frame


