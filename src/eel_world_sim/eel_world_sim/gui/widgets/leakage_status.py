"""Leakage status readout from leakage/status."""

from __future__ import annotations

from typing import TYPE_CHECKING

import dearpygui.dearpygui as dpg

if TYPE_CHECKING:
    from eel_world_sim.gui.ros_node import WorldSimGuiNode


def build_leakage_status() -> None:
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
                dpg.add_text("Leakage status")
                dpg.add_text("leak: —", tag="leakage_label")


def tick_leakage_status(node: WorldSimGuiNode) -> None:
    if dpg.does_item_exist("leakage_label"):
        leak = "yes" if node.leakage() else "no"
        dpg.set_value("leakage_label", f"leak: {leak}")
