class ImuSensor:
    def __init__(self, simulate=False):
        self.simulate = simulate
        if not simulate:
            import adafruit_bno055
            import board

            i2c = board.I2C()
            self.sensor = adafruit_bno055.BNO055_I2C(i2c)

    def get_heading(self):
        if self.simulate:
            return float(180)

        heading, roll, pitch = self.sensor.euler
        return float(heading or 0)

    def get_is_calibrated(self):
        if self.simulate:
            return False

        return self.sensor.calibrated or False

    def get_calibration_status(self):
        if self.simulate:
            return 1, 1, 1, 1

        sys, gyro, accel, mag = self.sensor.calibration_status
        return sys or 0, gyro or 0, accel or 0, mag or 0
