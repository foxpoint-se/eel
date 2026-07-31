from .types import CalibrationOffsets

PITCH_CORRECTION = 1.69
ROLL_CORRECTION = 0.0
HEADING_CORRECTION = 0


def get_corrected_pitch(pitch: float) -> float:
    return pitch + PITCH_CORRECTION


def get_corrected_roll(roll: float) -> float:
    return roll + ROLL_CORRECTION


# the sensor seems to be mounted 180 deg
def get_corrected_heading(heading: float) -> float:
    return (heading - 180 + HEADING_CORRECTION) % 360


def parse_euler_reading(
    heading: float | None,
    roll: float | None,
    pitch: float | None,
) -> tuple[float, float, float] | None:
    if heading is None or roll is None or pitch is None:
        return None
    return (
        get_corrected_heading(float(heading)),
        get_corrected_roll(float(roll)),
        get_corrected_pitch(-float(pitch)),
    )


def get_pitch_velocity(
    pitch: float,
    previous_pitch: float | None,
    now: float,
    previous_pitch_at: float | None,
) -> float:
    if previous_pitch is None or previous_pitch_at is None:
        return 0.0
    pitch_delta = pitch - previous_pitch
    time_delta = now - previous_pitch_at
    velocity = pitch_delta / time_delta
    return velocity


def update_pitch_velocity_state(
    *,
    euler: tuple[float, float, float] | None,
    now: float,
    previous_pitch: float | None,
    previous_pitch_at: float | None,
) -> tuple[float | None, float | None, float | None]:
    if euler is None:
        return None, None, None
    pitch = euler[2]
    velocity = get_pitch_velocity(pitch, previous_pitch, now, previous_pitch_at)
    return velocity, pitch, now


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
    def __init__(self) -> None:
        import adafruit_bno055
        import board

        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        # self.set_offset_values(CALIBRATION_1)
        self.sensor.offsets_magnetometer = (197, -106, 227)
        self.sensor.offsets_gyroscope = (-1, -5, 1)
        self.sensor.offsets_accelerometer = (-1, -32, -30)

    def get_is_calibrated(self) -> bool:
        return bool(self.sensor.calibrated)

    def get_calibration_status(self) -> tuple[int, int, int, int]:
        sys, gyro, accel, mag = self.sensor.calibration_status
        return sys or 0, gyro or 0, accel or 0, mag or 0

    def get_euler(self) -> tuple[float, float, float] | None:
        return parse_euler_reading(*self.sensor.euler)

    def get_calibration_offsets(self) -> CalibrationOffsets:
        mag = self.sensor.offsets_magnetometer
        gyr = self.sensor.offsets_gyroscope
        acc = self.sensor.offsets_accelerometer
        return {
            "mag": (mag[0], mag[1], mag[2]),
            "gyr": (gyr[0], gyr[1], gyr[2]),
            "acc": (acc[0], acc[1], acc[2]),
        }

    def set_offset_values(self, offset_mapping: CalibrationOffsets) -> None:
        self.sensor.offsets_magnetometer = offset_mapping["mag"]
        self.sensor.offsets_gyroscope = offset_mapping["gyr"]
        self.sensor.offsets_accelerometer = offset_mapping["acc"]
