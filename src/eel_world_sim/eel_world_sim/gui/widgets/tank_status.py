"""Tank front/rear status readout from tank_front/status and tank_rear/status."""

from __future__ import annotations

from typing import TYPE_CHECKING, NamedTuple

import dearpygui.dearpygui as dpg

if TYPE_CHECKING:
    from eel_world_sim.gui.ros_node import WorldSimGuiNode


class TankStatusView(NamedTuple):
    current_level: float
    target_level: float | None
    target_status: str
    is_autocorrecting: bool


def _format_target(target_level: float | None) -> str:
    if target_level is None:
        return "—"
    return f"{target_level:.2f}"


def build_tank_status() -> None:
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
                dpg.add_text("Tank status")
                dpg.add_text("front level: —", tag="tank_front_level_label")
                dpg.add_text("front target: —", tag="tank_front_target_label")
                dpg.add_text("rear level: —", tag="tank_rear_level_label")
                dpg.add_text("rear target: —", tag="tank_rear_target_label")


def tick_tank_status(node: WorldSimGuiNode) -> None:
    front = node.tank_front_status()
    rear = node.tank_rear_status()
    if dpg.does_item_exist("tank_front_level_label"):
        auto = " autocorrect" if front.is_autocorrecting else ""
        dpg.set_value(
            "tank_front_level_label",
            f"front: {front.current_level:.2f}  {front.target_status}{auto}",
        )
    if dpg.does_item_exist("tank_front_target_label"):
        dpg.set_value("tank_front_target_label", f"front target: {_format_target(front.target_level)}")
    if dpg.does_item_exist("tank_rear_level_label"):
        auto = " autocorrect" if rear.is_autocorrecting else ""
        dpg.set_value(
            "tank_rear_level_label",
            f"rear: {rear.current_level:.2f}  {rear.target_status}{auto}",
        )
    if dpg.does_item_exist("tank_rear_target_label"):
        dpg.set_value("tank_rear_target_label", f"rear target: {_format_target(rear.target_level)}")
