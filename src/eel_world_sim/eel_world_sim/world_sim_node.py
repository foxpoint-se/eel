"""World plant ROS node — owns dive + planar motion + tank fill + device stubs.

Topic names are hardcoded to match today's eel stack (temporary).
When the plant edge is clearer, replace with remaps/params.

Device stubs (battery/leakage/modem raw) copy old *_sim node behavior and rates.
"""

from math import cos, radians, sin
from time import time
from typing import Optional

import rclpy
from eel.motion.planar_motion import enu_yaw_deg_from_compass_bearing
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32
from tf2_ros import TransformBroadcaster

from eel_interfaces.msg import BatteryRaw, ImuOffsets, ImuRaw, ModemRaw
from eel_world_sim.attitude_model import AttitudeModel
from eel_world_sim.depth_model import DepthModel
from eel_world_sim.gnss_availability import should_publish_gnss_fix
from eel_world_sim.local_geo import meters_to_latlon
from eel_world_sim.modem_sim_math import simulated_registration_status, simulated_signal_strength
from eel_world_sim.motion_model import MotionModel
from eel_world_sim.tank_fill_model import TankFillModel

MOTOR_SETPOINT_TOPIC = "motor/setpoint"
RUDDER_STATUS_TOPIC = "rudder/status"
FRONT_TANK_PUMP_TOPIC = "tank_front/pump_setpoint"
REAR_TANK_PUMP_TOPIC = "tank_rear/pump_setpoint"
FRONT_TANK_LEVEL_TOPIC = "tank_front/level"
REAR_TANK_LEVEL_TOPIC = "tank_rear/level"
IMU_RAW_TOPIC = "imu/raw"
IMU_OFFSETS_TOPIC = "imu/offsets"
WORLD_DEPTH_TOPIC = "world/depth"
GNSS_FIX_TOPIC = "gnss/fix"
GNSS_FRAME_ID = "gnss_link"
ODOM_TOPIC = "odom"
ODOM_FRAME = "odom"
BASE_LINK_FRAME = "base_link"
BATTERY_RAW_TOPIC = "battery/raw"
LEAKAGE_RAW_TOPIC = "leakage/raw"
MODEM_RAW_TOPIC = "modem/raw"

PUBLISH_HZ = 10.0
BATTERY_PUBLISH_HZ = 2.0
LEAKAGE_PUBLISH_HZ = 1.0
MODEM_PUBLISH_PERIOD_S = 2.0
IMU_OFFSETS_PUBLISH_HZ = 0.5
BATTERY_MAX_VOLTAGE = 16.8


def _yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.z = sin(yaw_rad * 0.5)
    q.w = cos(yaw_rad * 0.5)
    return q


class WorldSimNode(Node):
    def __init__(self) -> None:
        super().__init__("world_sim")
        self._attitude = AttitudeModel()
        self._depth = DepthModel()
        self._motion = MotionModel()
        self._front_tank = TankFillModel()
        self._rear_tank = TankFillModel()
        self._last_step_at = time()
        self._battery_voltage_v = BATTERY_MAX_VOLTAGE
        self._battery_current_a = 2.56

        self.create_subscription(Float32, MOTOR_SETPOINT_TOPIC, self._on_motor_setpoint, 10)
        self.create_subscription(Vector3, RUDDER_STATUS_TOPIC, self._on_rudder_status, 10)
        self.create_subscription(Float32, FRONT_TANK_PUMP_TOPIC, self._on_front_pump, 10)
        self.create_subscription(Float32, REAR_TANK_PUMP_TOPIC, self._on_rear_pump, 10)

        self._depth_pub = self.create_publisher(Float32, WORLD_DEPTH_TOPIC, 10)
        self._imu_pub = self.create_publisher(ImuRaw, IMU_RAW_TOPIC, 10)
        self._imu_offsets_pub = self.create_publisher(ImuOffsets, IMU_OFFSETS_TOPIC, 10)
        self._gnss_fix_pub = self.create_publisher(NavSatFix, GNSS_FIX_TOPIC, 10)
        self._odom_pub = self.create_publisher(Odometry, ODOM_TOPIC, 10)
        self._front_level_pub = self.create_publisher(Float32, FRONT_TANK_LEVEL_TOPIC, 10)
        self._rear_level_pub = self.create_publisher(Float32, REAR_TANK_LEVEL_TOPIC, 10)
        self._battery_raw_pub = self.create_publisher(BatteryRaw, BATTERY_RAW_TOPIC, 10)
        self._leakage_raw_pub = self.create_publisher(Bool, LEAKAGE_RAW_TOPIC, 10)
        self._modem_raw_pub = self.create_publisher(ModemRaw, MODEM_RAW_TOPIC, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / PUBLISH_HZ, self._on_timer)
        self.create_timer(1.0 / BATTERY_PUBLISH_HZ, self._publish_battery_raw)
        self.create_timer(1.0 / LEAKAGE_PUBLISH_HZ, self._publish_leakage_raw)
        self.create_timer(MODEM_PUBLISH_PERIOD_S, self._publish_modem_raw)
        self.create_timer(1.0 / IMU_OFFSETS_PUBLISH_HZ, self._publish_imu_offsets)

        self.get_logger().info(
            "World physics started (attitude + depth + tank fill + odom + gnss/fix + imu/raw "
            "+ imu/offsets + battery/leakage/modem stubs)"
        )

    def _on_motor_setpoint(self, msg: Float32) -> None:
        cmd = float(msg.data)
        self._attitude.set_motor_cmd(cmd)
        self._depth.set_motor_cmd(cmd)
        self._motion.set_motor_cmd(cmd)

    def _on_rudder_status(self, msg: Vector3) -> None:
        self._attitude.set_rudder(float(msg.x), float(msg.y))

    def _on_front_pump(self, msg: Float32) -> None:
        self._front_tank.set_pump_cmd(float(msg.data))

    def _on_rear_pump(self, msg: Float32) -> None:
        self._rear_tank.set_pump_cmd(float(msg.data))

    def _publish_battery_raw(self) -> None:
        out = BatteryRaw()
        out.voltage_v = self._battery_voltage_v
        out.current_a = self._battery_current_a
        out.power_w = self._battery_voltage_v * self._battery_current_a
        out.supply_voltage_v = self._battery_voltage_v
        out.shunt_voltage_v = 0.0
        self._battery_raw_pub.publish(out)

    def _publish_leakage_raw(self) -> None:
        msg = Bool()
        msg.data = False
        self._leakage_raw_pub.publish(msg)

    def _publish_modem_raw(self) -> None:
        depth_m = self._depth.depth_m
        out = ModemRaw()
        out.reg_status = simulated_registration_status(depth_m)
        out.signal_strength = simulated_signal_strength(depth_m)
        self._modem_raw_pub.publish(out)

    def _publish_imu_offsets(self) -> None:
        # Same as old imu_sim: static zeros so GC still sees imu/offsets in sim.
        msg = ImuOffsets()
        msg.mag = [0, 0, 0]
        msg.gyr = [0, 0, 0]
        msg.acc = [0, 0, 0]
        self._imu_offsets_pub.publish(msg)

    def _on_timer(self) -> None:
        now = time()
        dt_s = now - self._last_step_at
        self._last_step_at = now

        front_level = self._front_tank.step(dt_s)
        rear_level = self._rear_tank.step(dt_s)
        self._attitude.set_tank_levels(front_level, rear_level)

        front_msg = Float32()
        front_msg.data = front_level
        self._front_level_pub.publish(front_msg)
        rear_msg = Float32()
        rear_msg.data = rear_level
        self._rear_level_pub.publish(rear_msg)

        heading_deg, pitch_deg = self._attitude.step(dt_s)
        self._depth.set_pitch_deg(pitch_deg)
        depth_m = self._depth.step(dt_s)
        self._motion.set_heading_deg(heading_deg)
        self._motion.set_pitch_deg(pitch_deg)
        x_m, y_m = self._motion.step(dt_s)

        depth_msg = Float32()
        depth_msg.data = depth_m
        self._depth_pub.publish(depth_msg)

        imu = ImuRaw()
        imu.is_calibrated = True
        imu.sys = 3
        imu.gyro = 3
        imu.accel = 3
        imu.mag = 3
        imu.heading = heading_deg
        imu.roll = 0.0
        imu.pitch = pitch_deg
        self._imu_pub.publish(imu)

        self._publish_gnss_fix(x_m, y_m, depth_m)
        self._publish_odom_and_tf(x_m, y_m, heading_deg)

    def _publish_gnss_fix(self, east_m: float, north_m: float, depth_m: float) -> None:
        if not should_publish_gnss_fix(depth_m):
            return

        lat, lon = meters_to_latlon(east_m, north_m)
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = GNSS_FRAME_ID
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = float("nan")
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self._gnss_fix_pub.publish(msg)

    def _publish_odom_and_tf(self, x_m: float, y_m: float, heading_deg: float) -> None:
        yaw_rad = radians(enu_yaw_deg_from_compass_bearing(heading_deg))
        orientation = _yaw_to_quaternion(yaw_rad)
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_LINK_FRAME
        odom.pose.pose.position.x = x_m
        odom.pose.pose.position.y = y_m
        odom.pose.pose.orientation = orientation
        self._odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = ODOM_FRAME
        transform.child_frame_id = BASE_LINK_FRAME
        transform.transform.translation.x = x_m
        transform.transform.translation.y = y_m
        transform.transform.rotation = orientation
        self._tf_broadcaster.sendTransform(transform)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WorldSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
