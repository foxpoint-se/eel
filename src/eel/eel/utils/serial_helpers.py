import serial
import threading
import time
from typing import Callable


class SerialReader(threading.Thread):
    def __init__(self, on_message: Callable[[], str], serial: str):
        super(SerialReader, self).__init__()
        self.daemon = True
        self.ser = serial
        self.on_message = on_message
        self.should_run = True
        self.start()

    def run(self):
        self.readlines()

    def pause(self):
        self.should_run = False

    def resume(self):
        self.should_run = True

    def readlines(self):
        while True:
            if self.should_run and self.on_message:
                line = self.ser.readline()
                if line:
                    message = line.decode("utf-8").strip()
                    self.on_message(message)
                time.sleep(0.01)


class SerialReaderWriter:
    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        timeout: int = 0,
        on_message: Callable[[], str] = None,
    ) -> None:
        self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        if on_message:
            self.reader = SerialReader(on_message, self.serial)

    def send(self, message) -> str:
        if message:
            self.serial.write(bytes("{}\n".format(message), encoding="utf-8"))
