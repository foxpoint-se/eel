"""Boat-centered chase view: boat fixed, local grid scrolls."""

from __future__ import annotations

import math

import dearpygui.dearpygui as dpg

from eel_world_sim.gui.constants import (
    CHASE_METERS_PER_GRID,
    CHASE_PIXELS_PER_METER,
    CHASE_VIEW_PX,
)


def build_chase_view() -> None:
    dpg.add_text("Chase")
    with dpg.drawlist(width=CHASE_VIEW_PX, height=CHASE_VIEW_PX, tag="chase_view_drawlist"):
        pass


def tick_chase_view(x_m: float, y_m: float, yaw_deg: float) -> None:
    if not dpg.does_item_exist("chase_view_drawlist"):
        return
    dpg.delete_item("chase_view_drawlist", children_only=True)

    dpg.draw_rectangle(
        (0, 0),
        (CHASE_VIEW_PX, CHASE_VIEW_PX),
        color=(50, 70, 50, 255),
        fill=(30, 45, 30, 255),
        parent="chase_view_drawlist",
    )

    cx = CHASE_VIEW_PX * 0.5
    cy = CHASE_VIEW_PX * 0.5
    spacing_px = CHASE_METERS_PER_GRID * CHASE_PIXELS_PER_METER
    offset_x = (-x_m * CHASE_PIXELS_PER_METER) % spacing_px
    offset_y = (y_m * CHASE_PIXELS_PER_METER) % spacing_px

    grid_color = (70, 100, 70, 255)
    x = offset_x - spacing_px
    while x < CHASE_VIEW_PX + spacing_px:
        dpg.draw_line((x, 0), (x, CHASE_VIEW_PX), color=grid_color, thickness=1, parent="chase_view_drawlist")
        x += spacing_px
    y = offset_y - spacing_px
    while y < CHASE_VIEW_PX + spacing_px:
        dpg.draw_line((0, y), (CHASE_VIEW_PX, y), color=grid_color, thickness=1, parent="chase_view_drawlist")
        y += spacing_px

    yaw_rad = math.radians(yaw_deg)
    half_len = 18.0
    half_thk = 7.0

    def rot(local_x: float, local_y: float) -> tuple[float, float]:
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)
        return (cx + local_x * c - local_y * s, cy - (local_x * s + local_y * c))

    dpg.draw_triangle(
        rot(half_len, 0.0),
        rot(-half_len, -half_thk),
        rot(-half_len, half_thk),
        color=(240, 200, 80, 255),
        fill=(240, 180, 60, 255),
        parent="chase_view_drawlist",
    )
    dpg.draw_text(
        (6, CHASE_VIEW_PX - 20),
        f"x {x_m:.0f} y {y_m:.0f} {yaw_deg:.0f}°",
        size=13,
        color=(220, 220, 220, 255),
        parent="chase_view_drawlist",
    )
