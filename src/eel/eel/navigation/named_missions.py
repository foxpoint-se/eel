from typing import Literal

NamedMissionName = Literal["rotholmen_runt_2025"]


def parse_named_mission(name: str) -> NamedMissionName | None:
    if name == "rotholmen_runt_2025":
        return "rotholmen_runt_2025"
    return None
