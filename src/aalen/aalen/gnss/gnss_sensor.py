import serial
import pynmea2
from .serial_reader import SerialReader

class GnssSensor:
    def __init__(self, simulate=False):
        self.current_lat = None
        self.current_lon = None
        self.simulate = simulate
        if not simulate:
            # TODO: remove this comment if we don't seem to have problem with timeout=0
            self.serial_source = serial.Serial("/dev/ttyUSB0", 9600, timeout=0)
            self.reader = SerialReader(self.__handle_readline, self.serial_source)
            self.reader.start()

    def __handle_readline(self, decoded_line):
        # TODO: remove this comment if GGA seems to work instead of GPRMC
        if 'GGA' in decoded_line:
            parsed = pynmea2.parse(decoded_line)
            lat = parsed.latitude
            lon = parsed.longitude
            self.__update_current_position(lat, lon)

    def __update_current_position(self, lat, lon):
        self.current_lat = lat
        self.current_lon = lon
    
    def get_current_position(self):
        if self.simulate:
            return 59.310132, 17.978479
        
        return self.current_lat, self.current_lon
