PITCH_CORRECTION = 1.69
ROLL_CORRECTION = 0.0
HEADING_CORRECTION = 0


def get_corrected_pitch(pitch: float):
    return pitch + PITCH_CORRECTION


def get_corrected_roll(roll: float):
    return roll + ROLL_CORRECTION


# the sensor seems to be mounted 180 deg
def get_corrected_heading(heading: float) -> float:
    return (heading - 180 + HEADING_CORRECTION) % 360


CALIBRATION_1 = {
    "mag": (193, 80, 84),
    "gyr": (-2, -7, 1),
    "acc": (-5, -13, -12),
}

CALIBRATION_2 = {
    "mag": (-32576, -32653, -32668),
    "gyr": (-1, -6, 0),
    "acc": (-14, -44, -34),
}

# Indoor calibration 1
# CALIBRATION COMPLETED
# Insert these preset offset values into project code:
#   Offsets_Magnetometer:  (193, 80, 84)
#   Offsets_Gyroscope:     (-2, -7, 1)
#   Offsets_Accelerometer: (-5, -13, -12)

# Indoor calibration 2
# CALIBRATION COMPLETED
# Insert these preset offset values into project code:
#   Offsets_Magnetometer:  (-32576, -32653, -32668)
#   Offsets_Gyroscope:     (-1, -6, 0)
#   Offsets_Accelerometer: (-14, -44, -34)


# @offsets_magnetometer.setter
# def offsets_magnetometer(self, offsets: Tuple[int, int, int]) -> None:
#     data = bytearray(6)
#     struct.pack_into("<hhh", data, 0, *offsets)
#     self._write_register(_OFFSET_MAGNET_REGISTER, bytes(data))

# offsets_gyroscope
# offsets_accelerometer

my_calibration = CALIBRATION_1

class ImuSensor:
    def __init__(self):
        import adafruit_bno055
        import board

        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.sensor.offsets_magnetometer = my_calibration["mag"]
        self.sensor.offsets_gyroscope = my_calibration["gyr"]
        self.sensor.offsets_accelerometer = my_calibration["acc"]

    def get_is_calibrated(self):
        return self.sensor.calibrated or False

    def get_calibration_status(self):
        sys, gyro, accel, mag = self.sensor.calibration_status
        return sys or 0, gyro or 0, accel or 0, mag or 0

    def get_euler(self):
        heading, roll, pitch = self.sensor.euler
        heading = float(heading or 0)
        heading = get_corrected_heading(heading)
        roll = get_corrected_roll(float(roll or 0))
        pitch = get_corrected_pitch(-float(pitch or 0))
        return heading, roll, pitch
