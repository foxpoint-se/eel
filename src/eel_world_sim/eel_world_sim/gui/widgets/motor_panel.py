"""Motor on/off controls."""

from __future__ import annotations

from typing import TYPE_CHECKING

import dearpygui.dearpygui as dpg

from eel_world_sim.gui.constants import MOTOR_OFF, MOTOR_ON

if TYPE_CHECKING:
    from eel_world_sim.gui.ros_node import WorldSimGuiNode


def build_motor_panel(node: WorldSimGuiNode) -> None:
    dpg.add_text("Motor")
    with dpg.group(horizontal=True):
        dpg.add_button(label="On", width=70, callback=lambda: node.set_motor(MOTOR_ON))
        dpg.add_button(label="Off", width=70, callback=lambda: node.set_motor(MOTOR_OFF))
    dpg.add_text("cmd: 0.0", tag="motor_label")


def tick_motor_panel(node: WorldSimGuiNode) -> None:
    if dpg.does_item_exist("motor_label"):
        dpg.set_value("motor_label", f"cmd: {node.motor_cmd():.1f}")
