"""Shared layout and drawing constants for the world-sim GUI."""

MOTOR_ON = 1.0
MOTOR_OFF = 0.0
MAX_DEPTH_M = 10.0

# Compact panel sizes so map + chase + side fit one row on a laptop.
VIEWPORT_W = 1280
VIEWPORT_H = 720
WINDOW_W = 1260
WINDOW_H = 700
CONTROLS_WIDTH = 260

CHASE_VIEW_PX = 240
SIDE_VIEW_WIDTH = 200
SIDE_VIEW_HEIGHT = 240
SIDE_METERS_PER_GRID = 2.0
SIDE_PIXELS_PER_METER = 18.0
MAP_VIEW_PX = 360

CHASE_METERS_PER_GRID = 2.0
CHASE_PIXELS_PER_METER = 18.0

MAP_SIZE_M = 500.0
MAP_HALF_M = MAP_SIZE_M * 0.5
MAP_GRID_STEP_M = 50.0
MAP_ZOOM_MIN_M = 50.0
MAP_ZOOM_MAX_M = MAP_SIZE_M
MAP_ZOOM_DEFAULT_M = MAP_SIZE_M

TRAIL_MAX_POINTS = 2000
TRAIL_MIN_STEP_M = 1.0
SURFACE_PAD_PX = 20
BOTTOM_PAD_PX = 20
CMD_REPUBLISH_HZ = 10.0
RUDDER_INDICATOR_SIZE = 72

# Island outlines in east/north meters from the Gröndal map origin (±250 m patch).
ISLANDS_M: tuple[tuple[tuple[float, float], ...], ...] = (
    (
        (60.0, 40.0),
        (110.0, 30.0),
        (130.0, 70.0),
        (90.0, 100.0),
        (50.0, 80.0),
    ),
    (
        (-175.0, -100.0),
        (-125.0, -140.0),
        (-90.0, -110.0),
        (-100.0, -60.0),
        (-150.0, -50.0),
    ),
)
