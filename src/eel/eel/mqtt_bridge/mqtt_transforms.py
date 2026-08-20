"""MQTT payload transforms shared by the bridge node."""

from typing import Callable, List, TypedDict

from std_msgs.msg import Bool

from eel_interfaces.msg import (
    BatteryStatus,
    ImuOffsets,
    ImuStatus,
    NavigationStatus,
    PressureStatus,
    TankStatus,
)

from .types import BoolMsgMqtt, CoordinateMqtt, transform_coordinate_msg


class ImuStatusMqtt(TypedDict):
    is_calibrated: bool
    sys: int
    gyro: int
    accel: int
    mag: int
    heading: float
    roll: float
    pitch: float
    pitch_velocity: float


class ImuOffsetsMqtt(TypedDict):
    mag: List[int]
    gyr: List[int]
    acc: List[int]


class BatteryStatusMqtt(TypedDict):
    voltage_ratio: float


class NavigationStatusMqtt(TypedDict):
    meters_to_target: float
    auto_mode_enabled: bool
    waypoints_left: List[CoordinateMqtt]
    count_goals_left: int
    mission_total_meters: float


class TankStatusMqtt(TypedDict):
    current_level: float
    target_level: List[float]
    target_status: str
    is_autocorrecting: bool


class PressureStatusMqtt(TypedDict):
    depth: float
    depth_velocity: float


SubscriberCallback = Callable[[str, bytes, bool, object, bool], None]


def transform_battery_msg(msg: BatteryStatus) -> BatteryStatusMqtt:
    return {"voltage_ratio": msg.voltage_ratio}


def transform_imu_msg(msg: ImuStatus) -> ImuStatusMqtt:
    return {
        "accel": msg.accel,
        "gyro": msg.gyro,
        "heading": msg.heading,
        "is_calibrated": msg.is_calibrated,
        "mag": msg.mag,
        "pitch": msg.pitch,
        "pitch_velocity": msg.pitch_velocity,
        "roll": msg.roll,
        "sys": msg.sys,
    }


def transform_imu_offsets_msg(msg: ImuOffsets) -> ImuOffsetsMqtt:
    return {
        "mag": [v for v in msg.mag],
        "acc": [v for v in msg.acc],
        "gyr": [v for v in msg.gyr],
    }


def transform_bool_msg(msg: Bool) -> BoolMsgMqtt:
    return {"data": msg.data}


def transform_nav_status(msg: NavigationStatus) -> NavigationStatusMqtt:
    return {
        "auto_mode_enabled": msg.auto_mode_enabled,
        "count_goals_left": msg.count_goals_left,
        "meters_to_target": msg.meters_to_target,
        "mission_total_meters": msg.mission_total_meters,
        "waypoints_left": [transform_coordinate_msg(w) for w in msg.waypoints_left],
    }


def transform_tank_status_msg(msg: TankStatus) -> TankStatusMqtt:
    return {
        "current_level": msg.current_level,
        "is_autocorrecting": msg.is_autocorrecting,
        "target_level": [float(t) for t in msg.target_level],
        "target_status": msg.target_status,
    }


def transform_pressure_status_msg(msg: PressureStatus) -> PressureStatusMqtt:
    return {"depth": msg.depth, "depth_velocity": msg.depth_velocity}
