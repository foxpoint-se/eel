"""Battery status readout (voltage / charge / current)."""

from __future__ import annotations

from typing import TYPE_CHECKING, NamedTuple

import dearpygui.dearpygui as dpg

if TYPE_CHECKING:
    from eel_world_sim.gui.ros_node import WorldSimGuiNode


class BatteryStatusView(NamedTuple):
    voltage_v: float
    current_a: float
    power_w: float
    voltage_ratio: float


def build_battery_status() -> None:
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
                dpg.add_text("Battery status")
                dpg.add_text("voltage: —", tag="battery_voltage_label")
                dpg.add_text("charge: —", tag="battery_charge_label")
                dpg.add_text("current / power: —", tag="battery_load_label")


def tick_battery_status(node: WorldSimGuiNode) -> None:
    battery = node.battery_status()
    if dpg.does_item_exist("battery_voltage_label"):
        dpg.set_value("battery_voltage_label", f"voltage: {battery.voltage_v:.2f} V")
    if dpg.does_item_exist("battery_charge_label"):
        dpg.set_value("battery_charge_label", f"charge: {battery.voltage_ratio * 100.0:.0f}%")
    if dpg.does_item_exist("battery_load_label"):
        dpg.set_value(
            "battery_load_label",
            f"current / power: {battery.current_a:.2f} A / {battery.power_w:.1f} W",
        )
