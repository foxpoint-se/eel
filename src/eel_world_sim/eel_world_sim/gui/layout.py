"""Compose the world-sim GUI layout."""

from __future__ import annotations

from typing import TYPE_CHECKING

import dearpygui.dearpygui as dpg

from eel_world_sim.gui.constants import CONTROLS_WIDTH, VIEWPORT_H, VIEWPORT_W, WINDOW_H, WINDOW_W
from eel_world_sim.gui.widgets.battery_status import build_battery_status, tick_battery_status
from eel_world_sim.gui.widgets.chase_view import build_chase_view, tick_chase_view
from eel_world_sim.gui.widgets.depth_control import build_depth_control, tick_depth_control
from eel_world_sim.gui.widgets.imu_status import build_imu_status, tick_imu_status
from eel_world_sim.gui.widgets.leakage_status import build_leakage_status, tick_leakage_status
from eel_world_sim.gui.widgets.map_view import build_map_view, tick_map_view
from eel_world_sim.gui.widgets.motor_panel import build_motor_panel, tick_motor_panel
from eel_world_sim.gui.widgets.nav_status import build_nav_status, tick_nav_status
from eel_world_sim.gui.widgets.rudder_panel import build_rudder_panel, tick_rudder_panel
from eel_world_sim.gui.widgets.rudder_status import build_rudder_status, tick_rudder_status
from eel_world_sim.gui.widgets.side_view import build_side_view, tick_side_view
from eel_world_sim.gui.widgets.tank_status import build_tank_status, tick_tank_status

if TYPE_CHECKING:
    from eel_world_sim.gui.ros_node import WorldSimGuiNode


def build_ui(node: WorldSimGuiNode) -> None:
    dpg.create_context()
    dpg.create_viewport(title="eel world sim", width=VIEWPORT_W, height=VIEWPORT_H)

    with dpg.window(label="world_sim_gui", tag="main", width=WINDOW_W, height=WINDOW_H):
        # One row: controls | map | chase | depth.
        with dpg.group(horizontal=True):
            with dpg.child_window(
                width=CONTROLS_WIDTH,
                border=False,
                autosize_y=True,
                auto_resize_y=True,
                no_scrollbar=True,
            ):
                build_motor_panel(node)
                dpg.add_spacer(height=8)
                build_rudder_panel(node)
                dpg.add_spacer(height=8)
                build_rudder_status()
                dpg.add_spacer(height=8)
                build_imu_status()
                dpg.add_spacer(height=8)
                build_battery_status()
                dpg.add_spacer(height=8)
                build_leakage_status()
                dpg.add_spacer(height=8)
                build_nav_status()
                dpg.add_spacer(height=8)
                build_tank_status()
                dpg.add_spacer(height=8)
                build_depth_control(node)

            dpg.add_spacer(width=8)
            with dpg.group():
                build_map_view(node)

            dpg.add_spacer(width=8)
            with dpg.group():
                build_chase_view()
                dpg.add_spacer(height=8)
                build_side_view()

    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("main", True)


def tick_ui(node: WorldSimGuiNode) -> None:
    tick_motor_panel(node)
    tick_rudder_panel(node)
    tick_rudder_status(node)
    tick_imu_status(node)
    tick_battery_status(node)
    tick_leakage_status(node)
    tick_nav_status(node)
    tick_tank_status(node)
    tick_depth_control(node)
    x_m, y_m, yaw_deg = node.odom_pose()
    east_m, north_m, map_yaw_deg, has_gnss = node.gnss_map_pose()
    tick_map_view(
        east_m,
        north_m,
        map_yaw_deg,
        has_gnss,
        node.trail(),
        node.show_live_trail(),
        node.follow_boat(),
        node.map_visible_m(),
        node.gnss_latlon(),
    )
    tick_chase_view(x_m, y_m, yaw_deg)
    tick_side_view(node.depth_m(), node.pitch_deg(), x_m, y_m)
