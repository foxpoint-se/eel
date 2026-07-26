#!/usr/bin/env python3
"""Sync package version from root pyproject.toml into setup.py and package.xml files."""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PYPROJECT = REPO_ROOT / "pyproject.toml"

TARGETS = (
    REPO_ROOT / "src" / "eel" / "setup.py",
    REPO_ROOT / "src" / "eel" / "package.xml",
    REPO_ROOT / "src" / "eel_interfaces" / "package.xml",
    REPO_ROOT / "src" / "eel_bringup" / "package.xml",
)


def read_project_version(pyproject: Path) -> str:
    text = pyproject.read_text(encoding="utf-8")
    in_project = False
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("[") and stripped.endswith("]"):
            in_project = stripped == "[project]"
            continue
        if not in_project:
            continue
        match = re.fullmatch(r'version\s*=\s*"([^"]+)"', stripped)
        if match:
            return match.group(1)
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


def main() -> int:
    try:
        version = read_project_version(PYPROJECT)
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    changed: list[Path] = []
    try:
        for path in TARGETS:
            if path.suffix == ".py":
                did_change = sync_setup_py(path, version)
            else:
                did_change = sync_package_xml(path, version)
            if did_change:
                changed.append(path)
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
    sys.exit(main())
