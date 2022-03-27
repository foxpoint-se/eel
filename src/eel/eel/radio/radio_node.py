#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
from types import SimpleNamespace
from eel_interfaces.msg import GnssStatus, ImuStatus, NavigationStatus
from std_msgs.msg import String, Float32, Bool
from ..utils.serial_helpers import SerialReaderWriter
from ..utils.radio_helpers.eel_side import (
    EelState,
    from_json_to_command,
    from_state_to_json,
)
from ..utils.topics import (
    RUDDER_CMD,
    MOTOR_CMD,
    IMU_STATUS,
    GNSS_STATUS,
    RADIO_IN,
    RADIO_OUT,
    NAVIGATION_STATUS,
    NAVIGATION_CMD,
)
from ..utils.constants import SIMULATE_PARAM


RUDDER_KEY = "rudder"
MOTOR_KEY = "motor"
NAV_KEY = "nav"
AUTO_MODE_KEY = "enable_auto_mode"

UPDATE_FREQUENCY = 1


def from_json(data):
    return json.loads(data, object_hook=lambda d: SimpleNamespace(**d))


# data = '{"name": "John Smith", "hometown": {"name": "New York", "id": 123}}'

# # Parse JSON into an object with attributes corresponding to dict keys.
# x = json.loads(data, object_hook=lambda d: SimpleNamespace(**d))
# print(x.name, x.hometown.name, x.hometown.id)


class RadioOutMessage:
    def set_position(self, position):
        self.position = position

    def set_imu_state(self, imu_state):
        self.imuState = imu_state

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__)


class Bar(object):
    def __init__(self, **kwargs):
        allowed_keys = {"a", "b", "c"}
        self.__dict__.update((k, v) for k, v in kwargs.items() if k in allowed_keys)
        # self.__dict__.update(kwargs)


# NOTE: keys are in camelcase to make them pretty in json
class ImuState:
    def __init__(self) -> None:
        self.c = False  # is calibrated
        self.s = 0  # system calibration value
        self.g = 0  # gyro calibration value
        self.a = 0  # accelerometer calibration value
        self.m = 0  # magnetometer calibration value
        self.h = None  # heading

    # def set_imu_state(self, imu_state):
    #     pass

    # def to_json(self):
    #     pass

    def is_valid(self):
        return True


class Position:
    def __init__(self) -> None:
        self.lt = None  # latitude
        self.ln = None  # longitude

    def is_valid(self):
        return self.lt and self.ln and self.lt != 0 and self.ln != 0


class NavState:
    def __init__(self) -> None:
        self.c = Position()  # target coordinate
        self.d = None  # distance to target
        self.t = None  # target tolerance
        self.a = None  # is auto mode enabled

    def is_valid(self):
        return True


class State:
    def __init__(self) -> None:
        self.p = Position()  # position
        self.n = NavState()  # navigation state
        self.i = ImuState()  # imu state

    def update_imu(self, msg):
        self.i.c = msg.is_calibrated
        self.i.s = msg.sys
        self.i.g = msg.gyro
        self.i.a = msg.accel
        self.i.m = msg.mag
        self.i.h = msg.euler_heading

    def update_position(self, msg):
        self.p.lt = msg.lat
        self.p.ln = msg.lon

    def update_nav(self, msg):
        if len(msg.next_target) > 0:
            next_target = msg.next_target[0]
            self.n.c.lt = next_target.lat
            self.n.c.ln = next_target.lon
            self.n.d = msg.meters_to_target
            self.n.t = msg.tolerance_in_meters
        else:
            self.n.c = Position()
            self.n.d = None
            self.n.t = None

        self.n.a = msg.auto_mode_enabled

    def to_json(self):
        # only_set_keys = {k: v for k, v in self.__dict__.items() if v is not None}
        # return json.dumps(only_set_keys)

        valid_keys = {
            k: v
            for k, v in self.__dict__.items()
            # if not hasattr(v, "is_valid") or v.is_valid()
            if v.is_valid()
            # if k == "position"
        }

        # return json.dumps(self, default=lambda o: o.__dict__)
        # return json.dumps(valid_keys, default=lambda o: o.__dict__)
        return json.dumps(valid_keys, default=lambda o: o.__dict__)


class Radio(Node):
    def __init__(self):
        super().__init__("radio_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value

        self.state = EelState()

        self.sender = self.create_timer(1.0 / UPDATE_FREQUENCY, self.send_state)

        self.imu_subscription = self.create_subscription(
            ImuStatus, IMU_STATUS, self.state.update_imu, 10
        )
        self.gnss_subscription = self.create_subscription(
            GnssStatus, GNSS_STATUS, self.state.update_gnss, 10
        )
        self.nav_subscription = self.create_subscription(
            NavigationStatus, NAVIGATION_STATUS, self.state.update_nav, 10
        )

        self.radio_out_publisher = self.create_publisher(String, RADIO_OUT, 10)
        self.radio_in_publisher = self.create_publisher(String, RADIO_IN, 10)
        self.rudder_publisher = self.create_publisher(Float32, RUDDER_CMD, 10)
        self.motor_publisher = self.create_publisher(Float32, MOTOR_CMD, 10)
        self.nav_publisher = self.create_publisher(Bool, NAVIGATION_CMD, 10)

        serial_port = (
            "/tmp/virtual_serial_eel" if self.should_simulate else "/dev/ttyS0"
        )
        self.reader_writer = SerialReaderWriter(
            serial_port, on_message=self.handle_incoming_message, timeout=None
        )

        self.get_logger().info(
            "{}Radio node started. Port: {}".format(
                "SIMULATE " if self.should_simulate else "", serial_port
            )
        )

    def send(self, message=None):
        if message:
            # self.get_logger().info(message)
            self.reader_writer.send(message)
            radio_out_msg = String()
            radio_out_msg.data = message
            self.radio_out_publisher.publish(radio_out_msg)

    # TODO: Either we validate the state kind of like this, or perhaps
    # we have a state variable that determines if the state has changed.
    # Or something else. In any case, this prevents the initial zeros
    # from being sent.
    def should_send_state(self):
        return self.state.get("lat") != 0 and self.state.get("lon") != 0
        # return False
        # return True

    # TODO: Either we validate the state kind of like this, or perhaps
    # we have a state variable that determines if the state has changed.
    # Or something else. In any case, this prevents the initial zeros
    # from being sent.
    def should_send_position(self):
        return (
            self.current_position
            and self.current_position.get("lat") != 0
            and self.current_position.get("lon") != 0
        )
        # return self.state.get("lat") != 0 and self.state.get("lon") != 0
        # return False
        # return True

    def send_state(self):
        # if self.should_send_state():
        #     self.send(message=json.dumps(self.state))

        # self.get_logger().info(self.state.t)

        # self.send(message=self.state.to_json())
        self.send(message=from_state_to_json(self.state))

    # def update_state(self, data):
    #     self.state = {**self.state, **data}

    def handle_incoming_message2(self, message):
        radio_in_msg = String()
        radio_in_msg.data = message
        self.radio_in_publisher.publish(radio_in_msg)
        data = None
        try:
            data = json.loads(message)
        except:
            self.get_logger().warning(
                "Could not parse json, incoming radio message: {}".format(message)
            )

        if data:
            if data.get(RUDDER_KEY) is not None:
                self.handle_rudder_msg(data.get(RUDDER_KEY))

            if data.get(MOTOR_KEY) is not None:
                self.handle_motor_msg(data.get(MOTOR_KEY))

            if data.get(NAV_KEY) is not None:
                self.handle_nav_radio_msg(data.get(NAV_KEY))

    def handle_incoming_message(self, message):
        self.get_logger().info(message)
        radio_in_msg = String()
        radio_in_msg.data = message
        self.radio_in_publisher.publish(radio_in_msg)
        data = None
        cmd = from_json_to_command(message)
        self.get_logger().info("haha" + str(cmd))
        if cmd.r != None:
            self.handle_rudder_msg(cmd.r)
        if cmd.m != None:
            self.handle_motor_msg(cmd.m)
        if cmd.a != None:
            self.handle_nav_radio_msg(cmd.a)
        # try:
        #     data = json.loads(message)
        # except:
        #     self.get_logger().warning(
        #         "Could not parse json, incoming radio message: {}".format(message)
        #     )

        # if data:
        #     if data.get(RUDDER_KEY) is not None:
        #         self.handle_rudder_msg(data.get(RUDDER_KEY))

        #     if data.get(MOTOR_KEY) is not None:
        #         self.handle_motor_msg(data.get(MOTOR_KEY))

        #     if data.get(NAV_KEY) is not None:
        #         self.handle_nav_radio_msg(data.get(NAV_KEY))

    def handle_nav_radio_msg(self, auto_mode_enabled):
        nav_msg = Bool()
        nav_msg.data = bool(auto_mode_enabled)
        self.nav_publisher.publish(nav_msg)

    def handle_nav_radio_msg2(self, msg):
        if msg.get(AUTO_MODE_KEY) is not None:
            nav_msg = Bool()
            nav_msg.data = bool(msg.get(AUTO_MODE_KEY))
            self.nav_publisher.publish(nav_msg)

    def handle_rudder_msg(self, rudder_msg_value):
        rudder_value = None
        try:
            rudder_value = float(rudder_msg_value)
        except:
            self.get_logger().warning(
                "Could not parse float, value {}".format(rudder_msg_value)
            )

        if rudder_value is not None:
            topic_msg = Float32()
            topic_msg.data = rudder_value
            self.rudder_publisher.publish(topic_msg)

    def handle_motor_msg(self, motor_msg_value):
        motor_value = None
        try:
            motor_value = float(motor_msg_value)
        except:
            self.get_logger().warning(
                "Could not parse float, value {}".format(motor_msg_value)
            )

        if motor_value is not None:
            topic_msg = Float32()
            topic_msg.data = motor_value
            self.motor_publisher.publish(topic_msg)

    def handle_nav_update(self, nav_msg):
        next_target = None
        if len(nav_msg.next_target) > 0 and nav_msg.auto_mode_enabled:
            coordinate = nav_msg.next_target[0]
            next_target = {
                "coordinate": {"lat": coordinate.lat, "lon": coordinate.lon},
                "distance": nav_msg.meters_to_target,
                "tolerance": nav_msg.tolerance_in_meters,
            }
        data = {"nextTarget": next_target, "autoModeEnabled": nav_msg.auto_mode_enabled}
        # self.update_state(data)


def main(args=None):
    rclpy.init(args=args)
    node = Radio()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
