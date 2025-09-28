import sys
from typing import Dict, List, Literal, TypedDict, Union
from launch import LaunchDescription
from launch_ros.actions import Node


class TankConfig(TypedDict):
    simulate: bool
    cmd_topic: Literal["tank_front/cmd", "tank_rear/cmd"]
    status_topic: Literal["tank_front/status", "tank_rear/status"]
    motor_pin: Literal[23, 24]
    direction_pin: Literal[18, 25]
    distance_sensor_channel: Literal[0, 1]
    tank_floor_value: float
    tank_ceiling_value: float

# ros2 run eel tank --ros-args -p cmd_topic:=tank_front/cmd -p status_topic:=tank_front/status -p motor_pin:=23 -p direction_pin:=18 -p distance_sensor_channel:=0 -p tank_floor_value:=0.647 -p tank_ceiling_value:=0.18

# more conservative floor and ceiling
# ros2 run eel tank --ros-args -p cmd_topic:=tank_front/cmd -p status_topic:=tank_front/status -p motor_pin:=23 -p direction_pin:=18 -p distance_sensor_channel:=0 -p tank_floor_value:=0.55 -p tank_ceiling_value:=0.3

front_tank_config: TankConfig = {
    "simulate": False,
    "cmd_topic": "tank_front/cmd",
    "status_topic": "tank_front/status",
    "motor_pin": 23,
    "direction_pin": 18,
    "distance_sensor_channel": 0,
    "tank_floor_value": 0.647,
    "tank_ceiling_value": 0.18,
}

# ros2 run eel tank --ros-args -p cmd_topic:=tank_rear/cmd -p status_topic:=tank_rear/status -p motor_pin:=24 -p direction_pin:=25 -p distance_sensor_channel:=1 -p tank_floor_value:=0.71 -p tank_ceiling_value:=0.29

# more conservative floor and ceiling
# ros2 run eel tank --ros-args -p cmd_topic:=tank_rear/cmd -p status_topic:=tank_rear/status -p motor_pin:=24 -p direction_pin:=25 -p distance_sensor_channel:=1 -p tank_floor_value:=0.6 -p tank_ceiling_value:=0.4
rear_tank_config: TankConfig = {
    "simulate": False,
    "cmd_topic": "tank_rear/cmd",
    "status_topic": "tank_rear/status",
    "motor_pin": 24,
    "direction_pin": 25,
    "distance_sensor_channel": 1,
    "tank_floor_value": 0.71,
    "tank_ceiling_value": 0.29,
}

# ros2 topic pub /depth_control/cmd eel_interfaces/msg/DepthControlCmd "{depth_target: 0.2, pitch_target: 0.0, depth_pid_type: 'hej', pitch_pid_type: 'hej'}"
# ros2 topic pub /depth_control/cmd eel_interfaces/msg/DepthControlCmd "{depth_target: 0.1, pitch_target: 0.0, depth_pid_type: 'hej', pitch_pid_type: 'hej'}"
# ros2 topic pub /depth_control/cmd eel_interfaces/msg/DepthControlCmd "{depth_target: 0.05, pitch_target: 10.0, depth_pid_type: 'hej', pitch_pid_type: 'hej'}"


# Notes for next time
# start tanks individually
# commands above in this file. we used the conservative ranges
# start tanks, depth control, pressure (while above surface) and imu
# monitor all topics, including tanks cmd to see what the pid commands
# to start tweaking next time, look at the depth control node:
# the mixing of depth and pitch, but most importantly the weird factors and constants we add to it
# maybe try without those completely, to start with and see what happens.
# besides not being able to reach depth or pitch, it hovers quite nicely :)

SIMULATE_PARAM = "simulate"
TANK_PARAM = "tank"

simulate_value = None
current_config = None

for arg in sys.argv:
    if arg.startswith(f"{SIMULATE_PARAM}:="):
        value = str(arg.split(":=")[1])
        if value == "false" or value == "False":
            simulate_value = False
        elif value == "true" or value == "True":
            simulate_value = True

    if arg.startswith(f"{TANK_PARAM}:="):
        value = str(arg.split(":=")[1])
        if value == "front":
            current_config = front_tank_config
        elif value == "rear":
            current_config = rear_tank_config


if simulate_value is None or current_config is None:
    raise AttributeError(
        f"Missing parameters. Please call with {SIMULATE_PARAM}:=<true | false> {TANK_PARAM}:=<front | rear>"
    )


current_config["simulate"] = simulate_value

ValidTypes = (str, bool, float, int)
LaunchParameters = List[Dict[str, Union[str, bool, float, int]]]

launch_parameters: LaunchParameters = []

for key, value in current_config.items():
    if not isinstance(value, ValidTypes):
        raise TypeError(
            f"{type(value)} is an invalid type. Valid types are {ValidTypes}. See {key=} {value=} "
        )
    param = {key: value}
    launch_parameters.append(param)


def generate_launch_description():
    ld = LaunchDescription()

    front_tank_node = Node(
        package="eel",
        executable="tank",
        name="front_tank",
        parameters=launch_parameters,
    )

    ld.add_action(front_tank_node)

    return ld
