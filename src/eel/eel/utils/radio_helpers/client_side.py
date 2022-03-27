import json
from .utils import instance_to_str


class ClientCoordinate:
    def __init__(self) -> None:
        self.lat = None
        self.lon = None

    def __str__(self) -> str:
        return instance_to_str(self)


class ClientNavState:
    def __init__(self) -> None:
        self.autoMode = None
        self.distance = None
        self.coordinate = None
        self.tolerance = None

    def __str__(self) -> str:
        return instance_to_str(self)


class ClientImuState:
    def __init__(self) -> None:
        self.heading = None
        self.gyro = None
        self.magnetometer = None
        self.accelerometer = None
        self.system = None

    def __str__(self) -> str:
        return instance_to_str(self)


class ClientEelState:
    def __init__(self) -> None:
        self.positions = []
        self.nav = ClientNavState()
        self.imu = ClientImuState()

    def update_eel_state(self, eel_state):
        if eel_state.n:
            self.update_nav(eel_state.n)
        if eel_state.i:
            self.update_imu(eel_state.i)
        if eel_state.g:
            self.update_gnss(eel_state.g)

    def update_nav(self, eel_state_nav):
        self.nav.autoMode = eel_state_nav.a
        self.nav.coordinate = (
            self.nav.coordinate if self.nav.coordinate else ClientCoordinate()
        )
        if eel_state_nav.c:
            self.nav.coordinate.lat = eel_state_nav.c.lt
            self.nav.coordinate.lon = eel_state_nav.c.ln
        else:
            self.nav.coordinate = None
        self.nav.distance = eel_state_nav.d
        self.nav.tolerance = eel_state_nav.t

    def update_imu(self, eel_state_imu):
        self.imu.heading = eel_state_imu.h
        self.imu.gyro = eel_state_imu.g
        self.imu.magnetometer = eel_state_imu.m
        self.imu.accelerometer = eel_state_imu.a
        self.imu.system = eel_state_imu.s

    def update_gnss(self, eel_state_gnss):
        coordinate = ClientCoordinate()
        coordinate.lat = eel_state_gnss.c.lt
        coordinate.lon = eel_state_gnss.c.ln
        self.positions.append(coordinate)

    def __str__(self) -> str:
        return instance_to_str(self)

    def to_dict(obj):
        return json.loads(json.dumps(obj, default=lambda o: o.__dict__))
