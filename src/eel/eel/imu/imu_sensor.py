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
        return float(heading or 0), float(roll or 0), float(pitch or 0)
