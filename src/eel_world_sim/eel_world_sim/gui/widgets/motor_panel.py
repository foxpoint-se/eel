"""Motor throttle controls."""

from __future__ import annotations

from typing import TYPE_CHECKING

import dearpygui.dearpygui as dpg

if TYPE_CHECKING:
    from eel_world_sim.gui.ros_node import WorldSimGuiNode


def build_motor_panel(node: WorldSimGuiNode) -> None:
    dpg.add_text("Motor")
    dpg.add_slider_int(
        tag="motor_slider",
        default_value=0,
        min_value=-100,
        max_value=100,
        width=220,
        callback=lambda: node.set_motor_slider(float(dpg.get_value("motor_slider"))),
    )
    dpg.add_text("cmd: 0.00", tag="motor_label")


def tick_motor_panel(node: WorldSimGuiNode) -> None:
    if dpg.does_item_exist("motor_label"):
        slider = float(dpg.get_value("motor_slider")) if dpg.does_item_exist("motor_slider") else 0.0
        dpg.set_value(
            "motor_label",
            f"slider: {slider:.0f}  cmd: {node.motor_cmd():.2f}",
        )
