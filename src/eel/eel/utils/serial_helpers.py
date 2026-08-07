import threading
import time
from typing import Callable, Optional

import serial
from rclpy.logging import get_logger

SLEEP_TIME = 0.01

logger = get_logger(__name__)


class SerialReaderWriter:
    def __init__(
        self,
        port: str,
        baudrate: int = 19200,
        timeout: int = 1,
        on_message: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._ser = serial.serial_for_url(port, baudrate=baudrate, timeout=timeout)
        self._on_message = on_message
        self._ser.flush()
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def send(self, message: str) -> None:
        self._write_one_message(message)

    def _write_one_message(self, message: str) -> None:
        msg_line = "{}\n".format(message)
        self._ser.write(bytes(msg_line, "utf-8"))
        self._ser.flush()

    def _loop(self) -> None:
        while True:
            msg = self._ser.readline()
            if msg and self._on_message is not None:
                try:
                    self._on_message(msg.decode("utf-8").strip())
                except UnicodeDecodeError:
                    logger.warning("Unicode decode error, ignoring serial line: %r", msg)

            self._ser.flush()
            time.sleep(SLEEP_TIME)
