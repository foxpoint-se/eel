"""Depth side-view panel with scrolling forward grid (like chase view)."""

from __future__ import annotations

import math

import dearpygui.dearpygui as dpg

from eel_world_sim.gui.constants import (
    BOTTOM_PAD_PX,
    MAX_DEPTH_M,
    SIDE_METERS_PER_GRID,
    SIDE_PIXELS_PER_METER,
    SIDE_VIEW_HEIGHT,
    SIDE_VIEW_WIDTH,
    SURFACE_PAD_PX,
)


def build_side_view() -> None:
    dpg.add_text("Depth")
    with dpg.drawlist(width=SIDE_VIEW_WIDTH, height=SIDE_VIEW_HEIGHT, tag="side_view_drawlist"):
        pass


def tick_side_view(depth_m: float, pitch_deg: float, x_m: float, y_m: float) -> None:
    if not dpg.does_item_exist("side_view_drawlist"):
        return
    dpg.delete_item("side_view_drawlist", children_only=True)

    dpg.draw_rectangle(
        (0, 0),
        (SIDE_VIEW_WIDTH, SIDE_VIEW_HEIGHT),
        color=(40, 60, 90, 255),
        fill=(20, 40, 70, 255),
        parent="side_view_drawlist",
    )

    # Boat stays horizontally centered; grid scrolls with along-track distance from odom.
    along_track_m = math.hypot(x_m, y_m)
    spacing_px = SIDE_METERS_PER_GRID * SIDE_PIXELS_PER_METER
    offset_x = (-along_track_m * SIDE_PIXELS_PER_METER) % spacing_px
    grid_color = (45, 75, 105, 255)

    x = offset_x - spacing_px
    while x < SIDE_VIEW_WIDTH + spacing_px:
        dpg.draw_line(
            (x, SURFACE_PAD_PX),
            (x, SIDE_VIEW_HEIGHT),
            color=grid_color,
            thickness=1,
            parent="side_view_drawlist",
        )
        x += spacing_px

    # Horizontal depth lines (fixed in the view, not scrolling).
    usable = SIDE_VIEW_HEIGHT - SURFACE_PAD_PX - BOTTOM_PAD_PX
    for depth_mark in (2.0, 4.0, 6.0, 8.0):
        gy = SURFACE_PAD_PX + (depth_mark / MAX_DEPTH_M) * usable
        dpg.draw_line((0, gy), (SIDE_VIEW_WIDTH, gy), color=grid_color, thickness=1, parent="side_view_drawlist")

    dpg.draw_line(
        (0, SURFACE_PAD_PX),
        (SIDE_VIEW_WIDTH, SURFACE_PAD_PX),
        color=(160, 200, 255, 255),
        thickness=2,
        parent="side_view_drawlist",
    )
    dpg.draw_text((6, 2), "surface", size=12, color=(180, 210, 255, 255), parent="side_view_drawlist")

    cx = SIDE_VIEW_WIDTH * 0.5
    cy = _depth_to_y(depth_m)
    half_len = 22.0
    half_thk = 7.0
    pitch_rad = math.radians(pitch_deg)

    def rot(x: float, y: float) -> tuple[float, float]:
        c = math.cos(pitch_rad)
        s = math.sin(pitch_rad)
        return (cx + x * c - y * s, cy + x * s + y * c)

    dpg.draw_triangle(
        rot(half_len, 0.0),
        rot(-half_len, -half_thk),
        rot(-half_len, half_thk),
        color=(240, 200, 80, 255),
        fill=(240, 180, 60, 255),
        parent="side_view_drawlist",
    )
    dpg.draw_text(
        (6, SIDE_VIEW_HEIGHT - 18),
        f"{depth_m:.2f} m  {pitch_deg:.0f}°",
        size=13,
        color=(220, 220, 220, 255),
        parent="side_view_drawlist",
    )


def _depth_to_y(depth_m: float) -> float:
    usable = SIDE_VIEW_HEIGHT - SURFACE_PAD_PX - BOTTOM_PAD_PX
    clamped = min(max(depth_m, 0.0), MAX_DEPTH_M)
    return SURFACE_PAD_PX + (clamped / MAX_DEPTH_M) * usable
