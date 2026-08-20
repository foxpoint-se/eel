"""IMU status readout (heading / attitude / calibration)."""

from __future__ import annotations

from typing import TYPE_CHECKING, NamedTuple

import dearpygui.dearpygui as dpg

if TYPE_CHECKING:
    from eel_world_sim.gui.ros_node import WorldSimGuiNode


class ImuStatusView(NamedTuple):
    heading_deg: float
    roll_deg: float
    pitch_deg: float
    pitch_velocity: float
    is_calibrated: bool
    sys: int
    gyro: int
    accel: int
    mag: int


def build_imu_status() -> None:
    with dpg.table(
        header_row=False,
        borders_outerH=True,
        borders_outerV=True,
        borders_innerH=False,
        borders_innerV=False,
    ):
        dpg.add_table_column()
        with dpg.table_row():
            with dpg.group():
                dpg.add_text("IMU status")
                dpg.add_text("heading: —", tag="imu_heading_label")
                dpg.add_text("roll / pitch: —", tag="imu_attitude_label")
                dpg.add_text("pitch vel: —", tag="imu_pitch_vel_label")
                dpg.add_text("cal: —", tag="imu_cal_label")


def tick_imu_status(node: WorldSimGuiNode) -> None:
    imu = node.imu_status()
    if dpg.does_item_exist("imu_heading_label"):
        dpg.set_value("imu_heading_label", f"heading: {imu.heading_deg:.1f}°")
    if dpg.does_item_exist("imu_attitude_label"):
        dpg.set_value("imu_attitude_label", f"roll / pitch: {imu.roll_deg:.1f}° / {imu.pitch_deg:.1f}°")
    if dpg.does_item_exist("imu_pitch_vel_label"):
        dpg.set_value("imu_pitch_vel_label", f"pitch vel: {imu.pitch_velocity:.2f}°/s")
    if dpg.does_item_exist("imu_cal_label"):
        cal = "yes" if imu.is_calibrated else "no"
        dpg.set_value(
            "imu_cal_label",
            f"cal: {cal}  sys/gyro/accel/mag {imu.sys}/{imu.gyro}/{imu.accel}/{imu.mag}",
        )
