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


class ImuSensor:
    def __init__(self):
        import adafruit_bno055
        import board

        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)

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
