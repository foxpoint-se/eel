"""Rudder status indicator (like ground-control XY vector)."""

from __future__ import annotations

from typing import TYPE_CHECKING

import dearpygui.dearpygui as dpg

from eel_world_sim.gui.constants import RUDDER_INDICATOR_SIZE

if TYPE_CHECKING:
    from eel_world_sim.gui.ros_node import WorldSimGuiNode


def build_rudder_status() -> None:
    dpg.add_text("Rudder status")
    with dpg.drawlist(
        width=RUDDER_INDICATOR_SIZE,
        height=RUDDER_INDICATOR_SIZE,
        tag="rudder_indicator_drawlist",
    ):
        pass
    dpg.add_text("x/y: 0.0 / 0.0", tag="rudder_status_label")


def tick_rudder_status(node: WorldSimGuiNode) -> None:
    rudder_x, rudder_y = node.rudder_status()
    rudder_x = _display_cmd(rudder_x)
    rudder_y = _display_cmd(rudder_y)
    if dpg.does_item_exist("rudder_status_label"):
        dpg.set_value("rudder_status_label", f"x/y: {rudder_x:.2f} / {rudder_y:.2f}")
    _draw_indicator(rudder_x, rudder_y)


def _display_cmd(value: float) -> float:
    """Avoid signed-zero flicker (−0.00 / +0.00) in the status label."""
    if abs(value) < 0.005:
        return 0.0
    return value


def _draw_indicator(rudder_x: float, rudder_y: float) -> None:
    if not dpg.does_item_exist("rudder_indicator_drawlist"):
        return
    dpg.delete_item("rudder_indicator_drawlist", children_only=True)

    size = RUDDER_INDICATOR_SIZE
    cx = size * 0.5
    cy = size * 0.5
    radius = size * 0.5 - 4.0

    dpg.draw_rectangle(
        (0, 0),
        (size, size),
        color=(80, 80, 80, 255),
        fill=(25, 25, 25, 255),
        parent="rudder_indicator_drawlist",
    )
    dpg.draw_circle((cx, cy), radius, color=(120, 120, 120, 255), thickness=1, parent="rudder_indicator_drawlist")
    tip_x = cx + rudder_x * radius
    tip_y = cy + rudder_y * radius
    dpg.draw_line((cx, cy), (tip_x, tip_y), color=(240, 200, 80, 255), thickness=3, parent="rudder_indicator_drawlist")
