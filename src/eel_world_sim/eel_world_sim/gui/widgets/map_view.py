"""Fixed geographic map: Gröndal origin ±250 m; pose markers from odom / gnss / localization."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import dearpygui.dearpygui as dpg

from eel_world_sim.gui.constants import (
    ISLANDS_M,
    MAP_GRID_STEP_M,
    MAP_HALF_M,
    MAP_MARKER_HALF_LEN_M,
    MAP_MARKER_MIN_HALF_LEN_PX,
    MAP_MARKER_WIDTH_RATIO,
    MAP_VIEW_PX,
    MAP_ZOOM_DEFAULT_M,
    MAP_ZOOM_MAX_M,
    MAP_ZOOM_MIN_M,
)
from eel_world_sim.local_geo import MAP_ORIGIN_LAT, MAP_ORIGIN_LON, format_latlon, meters_to_latlon

if TYPE_CHECKING:
    from eel_world_sim.gui.ros_node import WorldSimGuiNode


@dataclass(frozen=True)
class MapMarkerPose:
    east_m: float
    north_m: float
    yaw_deg: float
    available: bool


@dataclass(frozen=True)
class MapViewState:
    truth: MapMarkerPose
    gnss: MapMarkerPose
    localization: MapMarkerPose
    show_truth: bool
    show_gnss: bool
    show_localization: bool
    trail: list[tuple[float, float]]
    show_live_trail: bool
    follow_boat: bool
    visible_m: float


def build_map_view(node: WorldSimGuiNode) -> None:
    dpg.add_text(f"Map Gröndal {format_latlon(MAP_ORIGIN_LAT, MAP_ORIGIN_LON)}")
    dpg.add_text(f"Fixed patch ±{MAP_HALF_M:.0f} m east/north")
    with dpg.group(horizontal=True):
        dpg.add_button(label="-", width=36, callback=lambda: node.zoom_map_out())
        dpg.add_button(label="+", width=36, callback=lambda: node.zoom_map_in())
        dpg.add_button(label="Fit", width=44, callback=lambda: node.zoom_map_fit())
        dpg.add_checkbox(
            label="Live trail",
            default_value=True,
            tag="map_live_trail",
            callback=lambda: node.set_show_live_trail(bool(dpg.get_value("map_live_trail"))),
        )
        dpg.add_checkbox(
            label="Follow",
            default_value=True,
            tag="map_follow_boat",
            callback=lambda: node.set_follow_boat(bool(dpg.get_value("map_follow_boat"))),
        )
    with dpg.group(horizontal=True):
        dpg.add_checkbox(
            label=node.map_odom_topic(),
            default_value=True,
            tag="map_show_odom",
            callback=lambda: node.set_show_map_odom(bool(dpg.get_value("map_show_odom"))),
        )
    with dpg.group(horizontal=True):
        dpg.add_checkbox(
            label=node.map_gnss_topic(),
            default_value=True,
            tag="map_show_gnss",
            callback=lambda: node.set_show_map_gnss(bool(dpg.get_value("map_show_gnss"))),
        )
    with dpg.group(horizontal=True):
        dpg.add_checkbox(
            label=node.map_localization_topic(),
            default_value=True,
            tag="map_show_localization",
            callback=lambda: node.set_show_map_localization(bool(dpg.get_value("map_show_localization"))),
        )
    dpg.add_slider_float(
        tag="map_zoom_slider",
        default_value=MAP_ZOOM_DEFAULT_M,
        min_value=MAP_ZOOM_MIN_M,
        max_value=MAP_ZOOM_MAX_M,
        width=MAP_VIEW_PX,
        callback=lambda: node.set_map_visible_m(float(dpg.get_value("map_zoom_slider"))),
    )
    with dpg.drawlist(width=MAP_VIEW_PX, height=MAP_VIEW_PX, tag="map_view_drawlist"):
        pass


def tick_map_view(state: MapViewState) -> None:
    if dpg.does_item_exist("map_zoom_slider"):
        if abs(float(dpg.get_value("map_zoom_slider")) - state.visible_m) > 0.5:
            dpg.set_value("map_zoom_slider", state.visible_m)

    if not dpg.does_item_exist("map_view_drawlist"):
        return
    dpg.delete_item("map_view_drawlist", children_only=True)

    dpg.draw_rectangle(
        (0, 0),
        (MAP_VIEW_PX, MAP_VIEW_PX),
        color=(40, 80, 110, 255),
        fill=(20, 55, 85, 255),
        parent="map_view_drawlist",
    )

    if state.follow_boat and state.truth.available:
        center_east_m = state.truth.east_m
        center_north_m = state.truth.north_m
    else:
        center_east_m = 0.0
        center_north_m = 0.0
    half = state.visible_m * 0.5
    grid_color = (50, 90, 120, 255)
    step = MAP_GRID_STEP_M

    east = math.floor((center_east_m - half) / step) * step
    while east <= center_east_m + half + 0.1:
        x0, y0 = _enu_to_px(east, center_north_m - half, center_east_m, center_north_m, state.visible_m)
        x1, y1 = _enu_to_px(east, center_north_m + half, center_east_m, center_north_m, state.visible_m)
        dpg.draw_line((x0, y0), (x1, y1), color=grid_color, thickness=1, parent="map_view_drawlist")
        east += step

    north = math.floor((center_north_m - half) / step) * step
    while north <= center_north_m + half + 0.1:
        x0, y0 = _enu_to_px(center_east_m - half, north, center_east_m, center_north_m, state.visible_m)
        x1, y1 = _enu_to_px(center_east_m + half, north, center_east_m, center_north_m, state.visible_m)
        dpg.draw_line((x0, y0), (x1, y1), color=grid_color, thickness=1, parent="map_view_drawlist")
        north += step

    for island in ISLANDS_M:
        points = [_enu_to_px(ie, inorth, center_east_m, center_north_m, state.visible_m) for ie, inorth in island]
        dpg.draw_polygon(
            points, color=(40, 80, 40, 255), fill=(70, 120, 70, 255), thickness=1, parent="map_view_drawlist"
        )

    ox, oy = _enu_to_px(0.0, 0.0, center_east_m, center_north_m, state.visible_m)
    dpg.draw_line((ox - 8, oy), (ox + 8, oy), color=(220, 230, 240, 255), thickness=2, parent="map_view_drawlist")
    dpg.draw_line((ox, oy - 8), (ox, oy + 8), color=(220, 230, 240, 255), thickness=2, parent="map_view_drawlist")
    dpg.draw_text((ox + 10, oy - 18), "origin", size=12, color=(220, 230, 240, 255), parent="map_view_drawlist")

    if state.show_live_trail and len(state.trail) >= 2:
        trail_px = [_enu_to_px(te, tn, center_east_m, center_north_m, state.visible_m) for te, tn in state.trail]
        dpg.draw_polyline(trail_px, color=(255, 210, 90, 200), thickness=2, parent="map_view_drawlist")

    if state.show_gnss and state.gnss.available:
        _draw_boat_marker(
            state.gnss,
            center_east_m,
            center_north_m,
            state.visible_m,
            outline=(220, 90, 90, 140),
            fill=(200, 70, 70, 90),
        )
    if state.show_localization and state.localization.available:
        _draw_boat_marker(
            state.localization,
            center_east_m,
            center_north_m,
            state.visible_m,
            outline=(100, 180, 255, 140),
            fill=(70, 150, 230, 90),
        )
    if state.show_truth and state.truth.available:
        _draw_boat_marker(
            state.truth,
            center_east_m,
            center_north_m,
            state.visible_m,
            outline=(250, 250, 250, 255),
            fill=(240, 240, 240, 255),
        )

    _draw_corner_labels(center_east_m, center_north_m, state.visible_m)
    _draw_edge_meter_labels(center_east_m, center_north_m, state.visible_m)

    dpg.draw_text(
        (MAP_VIEW_PX * 0.5 - 6, 4),
        "N",
        size=14,
        color=(230, 230, 230, 255),
        parent="map_view_drawlist",
    )
    if state.truth.available:
        lat, lon = meters_to_latlon(state.truth.east_m, state.truth.north_m)
        status = f"{state.visible_m:.0f} m view  {format_latlon(lat, lon)}"
    else:
        status = f"{state.visible_m:.0f} m view  waiting for odom"
    dpg.draw_text(
        (6, MAP_VIEW_PX - 20),
        status,
        size=12,
        color=(220, 220, 220, 255),
        parent="map_view_drawlist",
    )


def _draw_boat_marker(
    pose: MapMarkerPose,
    center_east_m: float,
    center_north_m: float,
    visible_m: float,
    *,
    outline: tuple[int, int, int, int],
    fill: tuple[int, int, int, int],
) -> None:
    boat_px, boat_py = _enu_to_px(pose.east_m, pose.north_m, center_east_m, center_north_m, visible_m)
    px_per_m = MAP_VIEW_PX / visible_m
    half_len = max(MAP_MARKER_MIN_HALF_LEN_PX, MAP_MARKER_HALF_LEN_M * px_per_m)
    half_thk = half_len * MAP_MARKER_WIDTH_RATIO
    yaw_rad = math.radians(pose.yaw_deg)

    def rot(local_x: float, local_y: float) -> tuple[float, float]:
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)
        return (boat_px + local_x * c - local_y * s, boat_py - (local_x * s + local_y * c))

    dpg.draw_triangle(
        rot(half_len, 0.0),
        rot(-half_len, -half_thk),
        rot(-half_len, half_thk),
        color=outline,
        fill=fill,
        parent="map_view_drawlist",
    )


def _draw_corner_labels(center_east_m: float, center_north_m: float, visible_m: float) -> None:
    half = visible_m * 0.5
    corners = (
        (center_east_m - half, center_north_m + half, 4, 4),  # NW
        (center_east_m + half, center_north_m + half, MAP_VIEW_PX - 118, 4),  # NE
        (center_east_m - half, center_north_m - half, 4, MAP_VIEW_PX - 36),  # SW
        (center_east_m + half, center_north_m - half, MAP_VIEW_PX - 118, MAP_VIEW_PX - 36),  # SE
    )
    for east_m, north_m, px, py in corners:
        lat, lon = meters_to_latlon(east_m, north_m)
        dpg.draw_text(
            (px, py),
            format_latlon(lat, lon),
            size=11,
            color=(200, 220, 240, 255),
            parent="map_view_drawlist",
        )


def _draw_edge_meter_labels(center_east_m: float, center_north_m: float, visible_m: float) -> None:
    half = visible_m * 0.5
    ex, ey = _enu_to_px(center_east_m + half, center_north_m, center_east_m, center_north_m, visible_m)
    dpg.draw_text(
        (ex - 52, ey - 8),
        f"+{half:.0f} E",
        size=12,
        color=(180, 210, 230, 255),
        parent="map_view_drawlist",
    )
    nx, ny = _enu_to_px(center_east_m, center_north_m + half, center_east_m, center_north_m, visible_m)
    dpg.draw_text(
        (nx - 28, ny + 16),
        f"+{half:.0f} N",
        size=12,
        color=(180, 210, 230, 255),
        parent="map_view_drawlist",
    )


def _enu_to_px(
    east_m: float,
    north_m: float,
    center_east_m: float,
    center_north_m: float,
    visible_m: float,
) -> tuple[float, float]:
    half = visible_m * 0.5
    scale = MAP_VIEW_PX / visible_m
    px = (east_m - center_east_m + half) * scale
    py = (half - (north_m - center_north_m)) * scale
    return px, py
