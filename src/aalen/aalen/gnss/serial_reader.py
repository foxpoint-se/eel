import threading
import time

class SerialReader(threading.Thread):
    def __init__(self, on_readline=lambda l: print('UNHANDLED READLINE', l), serial=None):
        super(SerialReader, self).__init__()
        self.daemon = True
        self.ser = serial
        self.on_readline = on_readline
        self.should_run = True

    def run(self):
        self.readlines()

    def pause(self):
        self.should_run = False

    def resume(self):
        self.should_run = True

    def stop(self):
        self.should_run = False

    def readlines(self):
        while True:
            if self.should_run:
                line = self.ser.readline()
                if line:
                    string = line.decode('utf-8')
                    self.on_readline(string)
                time.sleep(0.01)
