"""Rudder yaw/pitch command controls."""

from __future__ import annotations

from typing import TYPE_CHECKING

import dearpygui.dearpygui as dpg

if TYPE_CHECKING:
    from eel_world_sim.gui.ros_node import WorldSimGuiNode


def build_rudder_panel(node: WorldSimGuiNode) -> None:
    dpg.add_text("Rudder")
    dpg.add_checkbox(
        label="manual control",
        default_value=True,
        tag="rudder_manual",
        callback=lambda: node.set_manual_rudder(bool(dpg.get_value("rudder_manual"))),
    )
    dpg.add_text("Rudder yaw (x)")
    with dpg.group(horizontal=True):
        dpg.add_button(label="L", width=40, callback=lambda: node.set_rudder_x(1.0))
        dpg.add_button(label="0", width=40, callback=lambda: node.set_rudder_x(0.0))
        dpg.add_button(label="R", width=40, callback=lambda: node.set_rudder_x(-1.0))
    dpg.add_slider_float(
        tag="rudder_x_slider",
        default_value=0.0,
        min_value=-1.0,
        max_value=1.0,
        width=220,
        callback=lambda: node.set_rudder_x(float(dpg.get_value("rudder_x_slider"))),
    )

    dpg.add_spacer(height=4)
    dpg.add_text("Rudder pitch (y)")
    with dpg.group(horizontal=True):
        dpg.add_button(label="Down", width=55, callback=lambda: node.set_rudder_y(1.0))
        dpg.add_button(label="0", width=40, callback=lambda: node.set_rudder_y(0.0))
        dpg.add_button(label="Up", width=55, callback=lambda: node.set_rudder_y(-1.0))
    dpg.add_slider_float(
        tag="rudder_y_slider",
        default_value=0.0,
        min_value=-1.0,
        max_value=1.0,
        width=220,
        callback=lambda: node.set_rudder_y(float(dpg.get_value("rudder_y_slider"))),
    )
    dpg.add_text("x/y: 0.0 / 0.0", tag="rudder_cmd_label")


def tick_rudder_panel(node: WorldSimGuiNode) -> None:
    if dpg.does_item_exist("rudder_cmd_label"):
        dpg.set_value(
            "rudder_cmd_label",
            f"x/y: {node.rudder_x_cmd():.2f} / {node.rudder_y_cmd():.2f}",
        )
