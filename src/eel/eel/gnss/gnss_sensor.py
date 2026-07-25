import pynmea2
from rclpy.logging import get_logger
from ..utils.serial_helpers import SerialReaderWriter

logger = get_logger(__name__)


class GnssSensor:
    def __init__(self, serial_port: str = "/dev/ttyUSB1") -> None:
        self.current_lat: float | None = None
        self.current_lon: float | None = None
        # TODO: remove this comment if we don't seem to have problem with timeout=0
        self.serial = SerialReaderWriter(
            serial_port, baudrate=9600, on_message=self.handle_message
        )

    def handle_message(self, message: str) -> None:
        # TODO: remove this comment if GGA seems to work instead of GPRMC
        if "GGA" in message:
            try:
                parsed = pynmea2.parse(message)
                lat = parsed.latitude
                lon = parsed.longitude
                self.update_current_position(lat, lon)
            except Exception:
                logger.info(f"could not parse gnss {message=}")

    def update_current_position(self, lat: float, lon: float) -> None:
        self.current_lat = lat
        self.current_lon = lon

    def get_current_position(self) -> tuple[float | None, float | None]:
        return self.current_lat, self.current_lon
