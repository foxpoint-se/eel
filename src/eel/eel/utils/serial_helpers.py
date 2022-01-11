import atexit
import sys
import serial
from typing import Callable
from serial.threaded import LineReader, ReaderThread


class ReaderWrapper(LineReader):
    def __init__(self):
        super(ReaderWrapper, self).__init__()
        self.on_message = None
        self.on_disconnected = None
        self.unhandled_messages = []

    TERMINATOR = b"\n"

    def connection_made(self, transport):
        super(ReaderWrapper, self).connection_made(transport)
        sys.stdout.write("Port opened.\n")

    def handle_line(self, data):
        if self.on_message:
            self.on_message(data)
        else:
            self.unhandled_messages.append(data)

    def connection_lost(self, exc):
        reason = None
        if exc:
            sys.stdout.write("{}\n".format(exc))
            reason = exc
        sys.stdout.write("Port closed.\n")
        if self.on_disconnected:
            self.on_disconnected(reason)


class SerialReaderWriter:
    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        timeout: int = 1,
        on_message: Callable[[str], None] = None,
        on_connected: Callable[[], None] = None,
        on_disconnected: Callable[[Exception], None] = None,
    ) -> None:
        self.ser = serial.serial_for_url(port, baudrate=baudrate, timeout=timeout)
        t = ReaderThread(self.ser, ReaderWrapper)
        t.start()
        transport, protocol = t.connect()
        if on_connected:
            on_connected()
        self.protocol = protocol
        self.protocol.on_message = on_message
        self.protocol.on_connected = on_connected
        self.protocol.on_disconnected = on_disconnected

        # Workaround, since `on_message` is set after `t.connect()`, but we need `protocol`
        # to be able to set `on_message` handler. This will get those messages.
        count = len(protocol.unhandled_messages)
        if on_message and count > 0:
            sys.stdout.write(
                "Handling unhandled messages right after connection: {}\n".format(count)
            )
            for m in protocol.unhandled_messages:
                on_message(m)

        atexit.register(t.close)

    def send(self, message):
        if message:
            self.protocol.write_line(message)
