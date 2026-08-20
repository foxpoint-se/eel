"""Per-boat wiring for boat.launch.py (ports + MQTT path). No simulate.

Modem serial stays hardcoded in hardware (/dev/ttyUSB4) — not listed here.

Historical notes (not separate profiles):
  Rotholmen compose used network_mode host + cyclonedds.xml (compose-only).
  Old tvalen-template left motor/rudder/battery/modem/mqtt commented out.
  Old rotholmen-alen had no tanks; rotholmen-tvalen mixed simulate:=true on some sensors.
  Old tvalen pressure mapped host /dev/ttyUSB0 -> container /dev/ttyUSB1; we use 1:1 mounts
  and the serial_port values the nodes were given.
"""

from typing import TypedDict


class BoatConfig(TypedDict):
    pressure_port: str
    gnss_port: str
    mqtt_config_path: str


_MQTT_FLEET = "/home/ubuntu/fleet/local_certs_and_config/iot_config.json"

ALEN: BoatConfig = {
    "pressure_port": "/dev/ttyUSB0",
    "gnss_port": "/dev/ttyUSB1",
    "mqtt_config_path": _MQTT_FLEET,
}

TVALEN: BoatConfig = {
    "pressure_port": "/dev/ttyUSB1",
    "gnss_port": "/dev/ttyUSB0",
    "mqtt_config_path": _MQTT_FLEET,
}


def boat_config(boat: str) -> BoatConfig:
    if boat == "alen":
        return ALEN
    if boat == "tvalen":
        return TVALEN
    raise ValueError(f"boat must be 'alen' or 'tvalen', got {boat!r}")
