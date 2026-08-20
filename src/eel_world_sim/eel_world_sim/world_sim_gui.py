"""Minimal world-sim GUI entrypoint.

No physics — publishes/subscribes only. Layout lives under eel_world_sim.gui.
"""

from __future__ import annotations

from typing import Optional

from eel_world_sim.gui.app import run


def main(args: Optional[list[str]] = None) -> None:
    run(args)


if __name__ == "__main__":
    main()
