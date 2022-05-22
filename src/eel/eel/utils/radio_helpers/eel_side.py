import json
from .utils import instance_to_str, to_json_filtered

# TODO: remove this
class ImuState:
    def __init__(self, c=None, s=None, g=None, a=None, m=None, e=None) -> None:
        self.c = c  # is calibrated
        self.s = s  # system calibration value
        self.g = g  # gyro calibration value
        self.a = a  # accelerometer calibration value
        self.m = m  # magnetometer calibration value
        self.e = e  # euler [heading, roll, pitch]

    def __str__(self) -> str:
        return instance_to_str(self)


class Coordinate:
    def __init__(self, lt=None, ln=None) -> None:
        self.lt = lt  # latitude
        self.ln = ln  # longitude

    def __str__(self) -> str:
        return instance_to_str(self)


class GnssState:
    def __init__(self, c=None) -> None:
        self.c = None if not c else Coordinate(**c)

    def __str__(self) -> str:
        return instance_to_str(self)


class NavState:
    def __init__(self, c=None, d=None, t=None, a=None) -> None:
        self.c = None if not c else Coordinate(**c)  # target coordinate
        self.d = d  # distance to target
        self.t = t  # target tolerance
        self.a = a  # is auto mode enabled

    def __str__(self) -> str:
        return instance_to_str(self)


class EelState:
    def __init__(self, n=None, i=None, g=None) -> None:
        self.n = None if not n else NavState(**n)
        self.i = None if not i else ImuState(**i)
        self.g = None if not g else GnssState(**g)

    def update_imu(self, msg):
        self.i = self.i if self.i else ImuState()
        self.i.c = msg.is_calibrated
        self.i.s = msg.sys
        self.i.g = msg.gyro
        self.i.a = msg.accel
        self.i.m = msg.mag
        self.i.e = msg.euler

    def update_gnss(self, msg):
        self.g = self.g if self.g else GnssState()
        self.g.c = self.g.c if self.g.c else Coordinate()
        self.g.c.lt = msg.lat
        self.g.c.ln = msg.lon

    def update_nav(self, msg):
        self.n = self.n if self.n else NavState()
        if len(msg.next_target) > 0:
            next_target = msg.next_target[0]
            self.n.c = self.n.c if self.n.c else Coordinate()
            self.n.c.lt = next_target.lat
            self.n.c.ln = next_target.lon
            self.n.d = msg.meters_to_target
            self.n.t = msg.tolerance_in_meters
        else:
            self.n.c = None
            self.n.d = None
            self.n.t = None

        self.n.a = msg.auto_mode_enabled

    def __str__(self) -> str:
        return instance_to_str(self)


def from_json_to_state(data):
    dict_converted = json.loads(data)
    return EelState(**dict_converted)


def from_state_to_json(state):
    return to_json_filtered(state)


class CommandMessage:
    def __init__(self, m=None, r=None, a=None) -> None:
        self.m = m
        self.r = r
        self.a = a

    def __str__(self) -> str:
        return instance_to_str(self)


def from_json_to_command(data):
    return json.loads(data, object_hook=lambda d: CommandMessage(**d))
