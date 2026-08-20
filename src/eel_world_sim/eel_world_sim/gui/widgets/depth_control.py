"""Depth-control enable toggle and target (from depth_control/status)."""

from __future__ import annotations

from typing import TYPE_CHECKING, NamedTuple

import dearpygui.dearpygui as dpg

if TYPE_CHECKING:
    from eel_world_sim.gui.ros_node import WorldSimGuiNode

DEPTH_TARGET_MAX_M = 3.0


class DepthControlStatusView(NamedTuple):
    is_enabled: bool
    depth_target_m: float


def build_depth_control(node: WorldSimGuiNode) -> None:
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
                dpg.add_text("Depth control")
                dpg.add_checkbox(
                    label="enabled",
                    default_value=False,
                    tag="depth_control_enabled",
                    callback=lambda: node.set_depth_control_enabled(bool(dpg.get_value("depth_control_enabled"))),
                )
                dpg.add_text("target: —", tag="depth_control_target_label")
                with dpg.group(horizontal=True):
                    dpg.add_input_float(
                        tag="depth_control_target_input",
                        default_value=0.0,
                        min_value=0.0,
                        max_value=DEPTH_TARGET_MAX_M,
                        step=0.1,
                        width=110,
                        on_enter=True,
                        callback=lambda: node.set_depth_target(float(dpg.get_value("depth_control_target_input"))),
                    )
                    dpg.add_button(
                        label="Set",
                        width=50,
                        callback=lambda: node.set_depth_target(float(dpg.get_value("depth_control_target_input"))),
                    )


def tick_depth_control(node: WorldSimGuiNode) -> None:
    status = node.depth_control_status()
    if dpg.does_item_exist("depth_control_enabled"):
        if bool(dpg.get_value("depth_control_enabled")) != status.is_enabled:
            dpg.set_value("depth_control_enabled", status.is_enabled)
    if dpg.does_item_exist("depth_control_target_label"):
        dpg.set_value("depth_control_target_label", f"target: {status.depth_target_m:.2f} m")
