"""Depth-control enable: idle until a depth cmd or enabled=true."""

from dataclasses import dataclass


@dataclass
class DepthEnable:
    is_enabled: bool = False

    def handle_depth_cmd(self) -> None:
        self.is_enabled = True

    def handle_enabled_cmd(self, desired: bool) -> None:
        self.is_enabled = desired
