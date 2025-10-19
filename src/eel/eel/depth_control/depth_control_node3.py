from typing import Optional
import rclpy
import math
from rclpy.node import Node
from ..utils.pid_controller import PidController
from std_msgs.msg import Float32
from eel_interfaces.msg import (
    DepthControlCmd,
    ImuStatus,
    PressureStatus,
)
from ..utils.topics import (
    DEPTH_CONTROL_CMD,
    IMU_STATUS,
    PRESSURE_STATUS,
    FRONT_TANK_CMD,
    REAR_TANK_CMD,
)
from ..utils.utils import clamp

# What we found for the real AUV
# FRONT_NEUTRAL = 0.45
# REAR_NEUTRAL = 0.69

# What is already hardcoded in simulation mode
FRONT_NEUTRAL = 0.5
REAR_NEUTRAL = 0.5


# more correct values below?
rho = 1000.0
g = 9.81
rf = 0.40
rr = 0.10
TANK_VOLUME_MAX = 0.00035

# conservative physical scaling (10× smaller than full-range)
Kb = 0.1 * (rho * g * 2.0 * TANK_VOLUME_MAX)  # previously ~6.87 N -> now ~0.687 N
Km = 0.1 * (
    rho * g * (rf + rr) * TANK_VOLUME_MAX
)  # previously ~1.72 N*m -> now ~0.172 N*m

Kb *= 3.0
Km *= 2.0

print(f"== Before == {Kb=} {Km=}")


# Suggested scalings (snappy but not saturating)
Kb = 3.0  # [N per PID unit]  (roughly 3 N unit)
Km = 0.6  # [N*m per PID unit]

# Depth controller — faster, small integrator
sim_depth_controller = PidController(
    set_point=0.0, kP=1.2, kI=0.02, kD=0.08, integrator_min=-0.4, integrator_max=0.4
)

# Pitch controller — stiffer, small integrator + damping
sim_pitch_controller = PidController(
    set_point=0.0, kP=1.0, kI=0.005, kD=0.35, integrator_min=-0.15, integrator_max=0.15
)


class DepthControlNode2(Node):
    def __init__(
        self,
        node_name: str = "depth_control2",
    ) -> None:
        super().__init__(node_name)
        self.depth_controller: Optional[PidController] = None
        self.pitch_controller: Optional[PidController] = None

        self.current_depth = 0.0
        self.current_pitch = 0.0

        self.create_subscription(
            DepthControlCmd, DEPTH_CONTROL_CMD, self.handle_cmd_msg, 10
        )

        self.create_subscription(ImuStatus, IMU_STATUS, self.handle_imu_msg, 10)
        self.create_subscription(
            PressureStatus, PRESSURE_STATUS, self.handle_pressure_msg, 10
        )

        self.front_tank_pub = self.create_publisher(Float32, FRONT_TANK_CMD, 10)
        self.rear_tank_pub = self.create_publisher(Float32, REAR_TANK_CMD, 10)
        self.create_timer(0.1, self.loop)
        self.get_logger().info("Depth control [3] started.")

    def handle_pressure_msg(self, msg: PressureStatus):
        self.current_depth = msg.depth

    def handle_imu_msg(self, msg: ImuStatus):
        self.current_pitch = math.radians(msg.pitch)

    def handle_cmd_msg(self, msg: DepthControlCmd) -> None:
        self.depth_target = msg.depth_target
        self.pitch_target = math.radians(msg.pitch_target)

        self.pitch_controller = sim_pitch_controller
        self.depth_controller = sim_depth_controller

        self.depth_controller.update_set_point(self.depth_target)
        self.pitch_controller.update_set_point(self.pitch_target)

    def pub_front(self, value: float) -> None:
        clamped = clamp(value, 0.0, 1.0)
        msg = Float32()
        msg.data = clamped
        self.front_tank_pub.publish(msg)

    def pub_rear(self, value: float) -> None:
        clamped = clamp(value, 0.0, 1.0)
        msg = Float32()
        msg.data = clamped
        self.rear_tank_pub.publish(msg)

    def loop(self) -> None:
        if not (self.pitch_controller and self.depth_controller):
            return

        depth_output = self.depth_controller.compute(
            self.current_depth
        )  # unitless [-1..1]
        pitch_output = self.pitch_controller.compute(
            self.current_pitch
        )  # unitless [-1..1]

        # pid outputs in SI (N and N*m)
        B_star = Kb * depth_output  # [N]
        M_star = Km * pitch_output  # [N*m]

        # convert forces/moments -> equivalent volumes
        B_vol = B_star / (rho * g)  # [m^3]
        M_vol = M_star / (rho * g)  # [m^3 * m]

        den = rf + rr + 1e-12
        delta_vf = (M_vol + rr * B_vol) / den
        delta_vr = (rf * B_vol - M_vol) / den

        # scale down deltas if they require too large a fraction of a tank
        allowed_fraction = 0.45
        frac_vf = abs(delta_vf) / TANK_VOLUME_MAX
        frac_vr = abs(delta_vr) / TANK_VOLUME_MAX
        max_frac = max(frac_vf, frac_vr, 1e-12)
        if max_frac > allowed_fraction:
            scale = allowed_fraction / max_frac
            delta_vf *= scale
            delta_vr *= scale
            self.get_logger().info(f"Scaling deltas by {scale:.3f} to avoid saturation")

        front_target = FRONT_NEUTRAL + (delta_vf / TANK_VOLUME_MAX)
        rear_target = REAR_NEUTRAL + (delta_vr / TANK_VOLUME_MAX)

        # Just logging
        err_pitch = self.pitch_controller.set_point - self.current_pitch
        err_depth = self.depth_controller.set_point - self.current_depth
        self.get_logger().info(
            f"eD={err_depth:.3f}m eP={math.degrees(err_pitch):.3f}deg "  # Convert back to degrees for logging
            f"pD={depth_output:.4f} pP={pitch_output:.4f} "
            f"curr_pitch={math.degrees(self.current_pitch):.2f}deg "
            f"target_pitch={math.degrees(self.pitch_target):.2f}deg "
            f"front_t={front_target:.3f} rear_t={rear_target:.3f}"
        )
        # End of logging
        self.pub_front(clamp(front_target, 0.0, 1.0))
        self.pub_rear(clamp(rear_target, 0.0, 1.0))


def main(args=None):
    rclpy.init(args=args)
    node = DepthControlNode2()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
