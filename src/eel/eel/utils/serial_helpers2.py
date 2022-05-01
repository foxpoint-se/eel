import serial
import time
from typing import Callable
import threading

SLEEP_TIME = 0.01


class SerialReaderWriter:
    def __init__(
        self,
        port: str,
        baudrate: int = 19200,
        timeout: int = 1,
        on_message: Callable[[str], None] = None,
    ):
        self._ser = serial.serial_for_url(port, baudrate=baudrate, timeout=timeout)
        self._on_message = on_message
        self._ser.flush()
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def send(self, message):
        self._write_one_message(message)

    # TODO: remove or use? it could be interesting to see if there's ever anything in in_waiting or out_waiting
    # def _flush_if_necessary(self):
    #     in_waiting = self._ser.in_waiting
    #     out_waiting = self._ser.out_waiting
    #     if in_waiting > 0 or out_waiting > 0:
    #         print("in_waiting", in_waiting, "out_waiting", out_waiting)
    #         self._ser.flush()

    def _write_one_message(self, message):
        msg_line = "{}\n".format(message)
        self._ser.write(bytes(msg_line, "utf-8"))
        self._ser.flush()
        # TODO: remove or use?
        # self._flush_if_necessary()

    def _loop(self):
        while True:
            msg = self._ser.readline()
            if msg:
                try:
                    self._on_message(msg.decode("utf-8").strip())
                except:
                    print("json decode error. ignoring", msg)

            self._ser.flush()
            # TODO: remove or use?
            # self._flush_if_necessary()
            time.sleep(SLEEP_TIME)
