"""Nav status readout from nav/status."""

from __future__ import annotations

from typing import TYPE_CHECKING, NamedTuple

import dearpygui.dearpygui as dpg

if TYPE_CHECKING:
    from eel_world_sim.gui.ros_node import WorldSimGuiNode

MISSION_STATUS_LABELS = {
    0: "waiting",
    1: "acquired",
    2: "started",
    3: "cancelled",
    4: "finished",
}


class NavStatusView(NamedTuple):
    auto_mode_enabled: bool
    mission_status: int
    meters_to_target: float
    mission_total_meters: float
    count_goals_left: int


def build_nav_status() -> None:
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
                dpg.add_text("Nav status")
                dpg.add_text("auto: —", tag="nav_auto_label")
                dpg.add_text("mission: —", tag="nav_mission_label")
                dpg.add_text("to target: —", tag="nav_to_target_label")
                dpg.add_text("mission length: —", tag="nav_length_label")
                dpg.add_text("goals left: —", tag="nav_goals_label")


def tick_nav_status(node: WorldSimGuiNode) -> None:
    nav = node.nav_status()
    if dpg.does_item_exist("nav_auto_label"):
        dpg.set_value("nav_auto_label", f"auto: {'yes' if nav.auto_mode_enabled else 'no'}")
    if dpg.does_item_exist("nav_mission_label"):
        label = MISSION_STATUS_LABELS.get(nav.mission_status, str(nav.mission_status))
        dpg.set_value("nav_mission_label", f"mission: {label}")
    if dpg.does_item_exist("nav_to_target_label"):
        dpg.set_value("nav_to_target_label", f"to target: {nav.meters_to_target:.1f} m")
    if dpg.does_item_exist("nav_length_label"):
        dpg.set_value("nav_length_label", f"mission length: {nav.mission_total_meters:.1f} m")
    if dpg.does_item_exist("nav_goals_label"):
        dpg.set_value("nav_goals_label", f"goals left: {nav.count_goals_left}")
