from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
SPEC = importlib.util.spec_from_file_location(
    "sync_version", SCRIPTS_DIR / "sync_version.py"
)
if SPEC is None or SPEC.loader is None:
    raise ImportError(f"Could not load sync_version from {SCRIPTS_DIR}")
sync_version = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(sync_version)


def test__when_project_header_has_comment__should_read_version(tmp_path: Path) -> None:
    pyproject = tmp_path / "pyproject.toml"
    pyproject.write_text(
        '[project]  # workspace\nversion = "1.2.3"  # bootstrap\n',
        encoding="utf-8",
    )

    assert sync_version.read_project_version(pyproject) == "1.2.3"


def test__when_version_uses_single_quotes__should_read_version(tmp_path: Path) -> None:
    pyproject = tmp_path / "pyproject.toml"
    pyproject.write_text("[project]\nversion = '2.0.0'\n", encoding="utf-8")

    assert sync_version.read_project_version(pyproject) == "2.0.0"


def test__when_setup_and_package_xml_stale__should_update_and_be_idempotent(
    tmp_path: Path,
) -> None:
    setup_py = tmp_path / "setup.py"
    package_xml = tmp_path / "package.xml"
    setup_py.write_text('setup(\n    version="0.0.0",\n)\n', encoding="utf-8")
    package_xml.write_text(
        "<package>\n  <version>0.0.0</version>\n</package>\n",
        encoding="utf-8",
    )

    changed = sync_version.sync_targets("0.1.0", (setup_py, package_xml))
    assert changed == [setup_py, package_xml]
    assert 'version="0.1.0"' in setup_py.read_text(encoding="utf-8")
    assert "<version>0.1.0</version>" in package_xml.read_text(encoding="utf-8")

    changed_again = sync_version.sync_targets("0.1.0", (setup_py, package_xml))
    assert changed_again == []


def test__when_setup_py_uses_single_quotes__should_preserve_quote_style(
    tmp_path: Path,
) -> None:
    setup_py = tmp_path / "setup.py"
    setup_py.write_text("setup(\n    version='0.0.0',\n)\n", encoding="utf-8")

    changed = sync_version.sync_targets("3.1.4", (setup_py,))
    assert changed == [setup_py]
    assert "version='3.1.4'" in setup_py.read_text(encoding="utf-8")


def test__when_project_version_missing__should_raise(tmp_path: Path) -> None:
    pyproject = tmp_path / "pyproject.toml"
    pyproject.write_text('[tool.mypy]\npython_version = "3.10"\n', encoding="utf-8")

    with pytest.raises(ValueError, match=r"No \[project\]\.version"):
        sync_version.read_project_version(pyproject)
