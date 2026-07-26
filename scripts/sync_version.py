#!/usr/bin/env python3
"""Sync package version from root pyproject.toml into setup.py and package.xml files."""

from __future__ import annotations

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PYPROJECT = REPO_ROOT / "pyproject.toml"

TARGETS = (
    REPO_ROOT / "src" / "eel" / "setup.py",
    REPO_ROOT / "src" / "eel" / "package.xml",
    REPO_ROOT / "src" / "eel_interfaces" / "package.xml",
    REPO_ROOT / "src" / "eel_bringup" / "package.xml",
)

_SECTION_HEADER = re.compile(r"^\[([^\]]+)\]")
_VERSION_ASSIGN = re.compile(r"""^version\s*=\s*(["'])([^"']+)\1\s*$""")


def _without_comment(line: str) -> str:
    """Drop a TOML `#` comment outside quotes (good enough for version lines)."""
    in_single = False
    in_double = False
    for i, char in enumerate(line):
        if char == "'" and not in_double:
            in_single = not in_single
        elif char == '"' and not in_single:
            in_double = not in_double
        elif char == "#" and not in_single and not in_double:
            return line[:i].rstrip()
    return line.rstrip()


def read_project_version(pyproject: Path) -> str:
    text = pyproject.read_text(encoding="utf-8")
    in_project = False
    for raw_line in text.splitlines():
        line = _without_comment(raw_line).strip()
        if not line:
            continue
        header = _SECTION_HEADER.fullmatch(line)
        if header:
            in_project = header.group(1) == "project"
            continue
        if not in_project:
            continue
        match = _VERSION_ASSIGN.fullmatch(line)
        if match:
            return match.group(2)
    raise ValueError(f"No [project].version found in {pyproject}")


def sync_setup_py(path: Path, version: str) -> bool:
    text = path.read_text(encoding="utf-8")
    updated, count = re.subn(
        r'(version\s*=\s*")[^"]+(")',
        rf"\g<1>{version}\2",
        text,
        count=1,
    )
    if count != 1:
        raise ValueError(f"Could not find version= in {path}")
    if updated == text:
        return False
    path.write_text(updated, encoding="utf-8")
    return True


def sync_package_xml(path: Path, version: str) -> bool:
    text = path.read_text(encoding="utf-8")
    updated, count = re.subn(
        r"(<version>)[^<]*(</version>)",
        rf"\g<1>{version}\2",
        text,
        count=1,
    )
    if count != 1:
        raise ValueError(f"Could not find <version> in {path}")
    if updated == text:
        return False
    path.write_text(updated, encoding="utf-8")
    return True


def sync_targets(version: str, targets: tuple[Path, ...] = TARGETS) -> list[Path]:
    changed: list[Path] = []
    for path in targets:
        if path.suffix == ".py":
            did_change = sync_setup_py(path, version)
        else:
            did_change = sync_package_xml(path, version)
        if did_change:
            changed.append(path)
    return changed


def main() -> int:
    import sys

    try:
        version = read_project_version(PYPROJECT)
        changed = sync_targets(version)
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"version {version}")
    if not changed:
        print("already in sync")
        return 0
    for path in changed:
        print(f"updated {path.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
