from typing import List, Optional
import rclpy
from rclpy.context import Context
import math
from rclpy.node import Node
from rclpy.parameter import Parameter
from ..utils.pid_controller import PidController
from std_msgs.msg import Float32
from eel_interfaces.msg import (
    DepthControlStatus,
    DepthControlCmd,
    ImuStatus,
    PressureStatus,
    TankStatus,
)
from ..utils.topics import (
    DEPTH_CONTROL_STATUS,
    DEPTH_CONTROL_CMD,
    IMU_STATUS,
    PRESSURE_STATUS,
    FRONT_TANK_CMD,
    REAR_TANK_CMD,
    FRONT_TANK_STATUS,
    REAR_TANK_STATUS,
)
from ..utils.utils import clamp

# Tank leverage factors (how much each tank affects pitch)
# FRONT_LEVERAGE = 1.0
# REAR_LEVERAGE = 0.5
# FRONT_LEVERAGE = 0.5
# REAR_LEVERAGE = 1.0
# FRONT_LEVERAGE = 1.0
# REAR_LEVERAGE = 1.0

FRONT_DISTANCE_FROM_CENTER = 1.0  # Front tank is 2x further from pivot
REAR_DISTANCE_FROM_CENTER = 1.0  # Rear tank is 1x distance (reference)
# FRONT_DISTANCE_FROM_CENTER = 0.6  # Front tank is 2x further from pivot
# REAR_DISTANCE_FROM_CENTER = 0.4  # Rear tank is 1x distance (reference)

# These should sum to 2.0 to maintain the math balance
# Or normalize them: total_leverage = FRONT_LEVERAGE + REAR_LEVERAGE

# What we found for the real AUV
# FRONT_NEUTRAL = 0.45
# REAR_NEUTRAL = 0.69

# What is already hardcoded in simulation mode
FRONT_NEUTRAL = 0.5
REAR_NEUTRAL = 0.5

average_neutral = (FRONT_NEUTRAL + REAR_NEUTRAL) / 2  # = 0.57
differential_neutral = FRONT_NEUTRAL - REAR_NEUTRAL  # = -0.24

"""
DEPTH CONTROL APPROACH OF THIS CONTROLLER:
- average_level controls overall buoyancy (depth)
- differential controls balance point (pitch) 
- Front tank = average + differential/2
- Rear tank = average - differential/2

This separates the math cleanly but controllers still interact.
Tuning: Start with pitch PID = 0, tune depth first, then add pitch.
"""

# sim_depth_controller = PidController(
#     set_point=0.0,
#     kP=0.8,
#     kI=0.0,  # 0.02,  # Kp/25 to eliminate steady-state error
#     kD=0.02,  # 0.5 * 22 / 4 = 2.75
# )

# sim_pitch_controller = PidController(
#     set_point=0.0,
#     # kP=0.5,
#     # kI=0.0,
#     # kD=0.01,
#     kP=1.0,
#     kI=0.01,
#     kD=2.0,
# )


# depth PID — moderate, some damping
sim_depth_controller = PidController(set_point=0.0, kP=0.6, kI=0.0, kD=0.05)
# sim_depth_controller = PidController(set_point=0.0, kP=2.0, kI=0.0, kD=0.5)

# pitch PID — remove integral for now, reduce D (damping), lower P slightly
sim_pitch_controller = PidController(set_point=0.0, kP=0.6, kI=0.0, kD=0.2)


rho = 1000  # kg/m3
g = 9.81  # m/s2

# rr = 0.35
# rf = 0.35

rf = 0.25
rr = 0.45

TANK_VOLUME_MAX = 0.000785

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

        # Kb = rho * g * 2 * TANK_VOLUME_MAX  # total buoyant range, ≈ 4.7 N
        # Km = rho * g * (rf + rr) * TANK_VOLUME_MAX  # ≈ 1.65 N·m
        # Kb = 4.7
        # Km = 1.65

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

    def loop_NEWER_BUT_OLD(self) -> None:
        if not (self.pitch_controller and self.depth_controller):
            return

        # --- Step 1: Compute PID outputs ---
        depth_output = self.depth_controller.compute(
            self.current_depth
        )  # unitless [-1..1]
        pitch_output = self.pitch_controller.compute(
            self.current_pitch
        )  # unitless [-1..1]

        # --- Step 2: Convert PID outputs → desired buoyant force & moment ---
        # These constants define how much effect 1.0 PID output means in physical terms

        # set Kb so PID output 1.0 -> total buoyancy equal to one tank full:
        Kb = rho * g * TANK_VOLUME_MAX  # ≈ 7.7 N
        # set Km so PID output 1.0 -> pitching moment that produces one tank differential:
        # Km = rho * g * (rf + rr) * TANK_VOLUME_MAX  # ≈ 5.4 N*m (with rf+rr=0.7)
        Km = (
            rho * g * (rf + rr) * TANK_VOLUME_MAX
        )  # 10% of max moment per unit PID output

        B_star = Kb * depth_output
        M_star = Km * pitch_output

        # convert B* and M* (in force units) to volumes
        B_vol = B_star / (rho * g)  # m^3
        M_vol = M_star / (rho * g)  # m^3 * m

        den = rf + rr + 1e-12
        delta_vf = 0.5 * (B_vol + M_vol / den)
        delta_vr = 0.5 * (B_vol - M_vol / den)

        # keep deltas achievable: scale if they would push fills outside [0,1]
        # allowed_fraction is how much of tank we allow to move from neutral (0..1)
        allowed_fraction = 0.45  # leaves a safety margin around 0.5 neutral

        # compute fraction of full tank each delta would require
        frac_vf = abs(delta_vf) / TANK_VOLUME_MAX
        frac_vr = abs(delta_vr) / TANK_VOLUME_MAX
        max_frac = max(frac_vf, frac_vr, 1e-12)

        if max_frac > allowed_fraction:
            scale = allowed_fraction / max_frac
            delta_vf *= scale
            delta_vr *= scale
            # optional debug log
            self.get_logger().info(f"Scaling deltas by {scale:.3f} to avoid saturation")

        front_target = FRONT_NEUTRAL + (delta_vf / TANK_VOLUME_MAX)
        rear_target = REAR_NEUTRAL + (delta_vr / TANK_VOLUME_MAX)

        err_pitch = self.pitch_controller.set_point - self.current_pitch
        err_depth = self.depth_controller.set_point - self.current_depth

        self.get_logger().info(
            f"eD={err_depth:.3f}m eP={math.degrees(err_pitch):.3f}deg "  # Convert back to degrees for logging
            f"pD={depth_output:.4f} pP={pitch_output:.4f} "
            f"curr_pitch={math.degrees(self.current_pitch):.2f}deg "
            f"target_pitch={math.degrees(self.pitch_target):.2f}deg "
            f"front_t={front_target:.3f} rear_t={rear_target:.3f}"
        )

        # --- Step 5: Clamp and publish ---
        self.pub_front(clamp(front_target, 0.0, 1.0))
        self.pub_rear(clamp(rear_target, 0.0, 1.0))

    def loop_OLD(self) -> None:
        if not (self.pitch_controller and self.depth_controller):
            return

        # --- Step 1: Compute PID outputs ---
        depth_output = self.depth_controller.compute(
            self.current_depth
        )  # unitless [-1..1]
        pitch_output = self.pitch_controller.compute(
            self.current_pitch
        )  # unitless [-1..1]

        # --- Step 2: Convert PID outputs → desired buoyant force & moment ---
        # These constants define how much effect 1.0 PID output means in physical terms
        Kb = 1.0  # buoyancy scaling (unitless -> tank delta)
        Km = 2.0  # pitch moment scaling (unitless -> tank delta * distance)

        B_star = Kb * depth_output
        M_star = Km * pitch_output

        # --- Step 3: Allocate to tanks ---
        # Tanks are located at known distances from the center
        Lf = FRONT_DISTANCE_FROM_CENTER
        Lr = REAR_DISTANCE_FROM_CENTER

        # Solve the linear system:
        # B* = ΔVf + ΔVr
        # M* = Lf*ΔVf - Lr*ΔVr
        # gives:
        det = Lf + Lr
        delta_front = (B_star / 2.0) + (M_star / (2.0 * det))
        delta_rear = (B_star / 2.0) - (M_star / (2.0 * det))

        # --- Step 4: Compute target tank fills ---
        front_target = FRONT_NEUTRAL + delta_front
        rear_target = REAR_NEUTRAL + delta_rear

        self.get_logger().info(
            f"err={self.current_pitch:.2f}, pout={pitch_output:.3f}, M*={M_star:.3f}, dF={delta_front:.3f}, dR={delta_rear:.3f}"
        )

        # --- Step 5: Clamp and publish ---
        self.pub_front(clamp(front_target, 0.0, 1.0))
        self.pub_rear(clamp(rear_target, 0.0, 1.0))


def main(args=None):
    rclpy.init(args=args)
    node = DepthControlNode2()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
