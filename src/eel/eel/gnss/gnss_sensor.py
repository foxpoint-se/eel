import pynmea2
from ..utils.serial_helpers import SerialReaderWriter


class GnssSensor:
    def __init__(self, simulate=False):
        self.current_lat = None
        self.current_lon = None
        self.simulate = simulate
        if not simulate:
            # TODO: remove this comment if we don't seem to have problem with timeout=0
            self.serial = SerialReaderWriter(
                "/dev/ttyUSB0", on_message=self.handle_message
            )

    def handle_message(self, message):
        # TODO: remove this comment if GGA seems to work instead of GPRMC
        if "GGA" in message:
            parsed = pynmea2.parse(message)
            lat = parsed.latitude
            lon = parsed.longitude
            self.update_current_position(lat, lon)

    def update_current_position(self, lat, lon):
        self.current_lat = lat
        self.current_lon = lon

    def get_current_position(self):
        if self.simulate:
            return 59.310132, 17.978479

        return self.current_lat, self.current_lon
