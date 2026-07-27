from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
SPEC = importlib.util.spec_from_file_location(
    "ci_discord_message", SCRIPTS_DIR / "ci_discord_message.py"
)
if SPEC is None or SPEC.loader is None:
    raise ImportError(f"Could not load ci_discord_message from {SCRIPTS_DIR}")
ci_discord_message = importlib.util.module_from_spec(SPEC)
import sys
sys.modules[SPEC.name] = ci_discord_message
SPEC.loader.exec_module(ci_discord_message)


REPO = "foxpoint-se/eel"
MERGE_MSG = "Merge pull request #204 from foxpoint-se/chore/verify-no-release"
SHA = "6161e61b1043b5c5fbff66d3a5a52e79e05db745"
RUN_URL = "https://github.com/foxpoint-se/eel/actions/runs/1"
RELEASE_URL = "https://github.com/foxpoint-se/eel/releases/tag/v1.0.1"


def _payload(**overrides: object) -> ci_discord_message.DiscordPayload:
    defaults = {
        "test_checks": "success",
        "build": "success",
        "release": "success",
        "released": False,
        "version": "1.0.1",
        "release_notes": "",
        "release_link": RELEASE_URL,
        "run_url": RUN_URL,
        "commit_sha": SHA,
        "commit_message": MERGE_MSG,
        "repo": REPO,
    }
    defaults.update(overrides)
    return ci_discord_message.build_discord_payload(**defaults)


def test__when_all_jobs_pass_and_no_release__should_build_no_version_message() -> None:
    payload = _payload()

    assert payload.title == "Eel CI — no new version"
    assert payload.status == "success"
    assert payload.url == RUN_URL
    assert "Build succeeded (still v1.0.1)" in payload.description
    assert "pull/204" in payload.description


def test__when_released__should_include_truncated_release_notes() -> None:
    notes = "### Bug Fixes\n- Fetch tags before checking out a release\n"
    payload = _payload(released=True, release_notes=notes)

    assert payload.title == "Released eel v1.0.1"
    assert payload.url == RELEASE_URL
    assert "Fetch tags before checking out a release" in payload.description
    assert "Full release notes" in payload.description
    assert "pull/204" in payload.description


def test__when_released_with_empty_notes__should_still_show_release_title() -> None:
    payload = _payload(released=True, release_notes="")

    assert payload.title == "Released eel v1.0.1"
    assert "Full release notes" in payload.description
    assert "###" not in payload.description


def test__when_test_checks_fail__should_list_failed_job() -> None:
    payload = _payload(test_checks="failure")

    assert payload.title == "Eel CI failed"
    assert payload.status == "failure"
    assert "Failed: checks" in payload.description


def test__when_job_cancelled__should_list_cancelled_job() -> None:
    payload = _payload(build="cancelled")

    assert payload.title == "Eel CI failed"
    assert "Failed: docker build" in payload.description


def test__when_released_without_release_link__should_fallback_to_run_url() -> None:
    payload = _payload(released=True, release_link="", release_notes="- fix something\n")

    assert payload.url == RUN_URL
    assert f"]({RUN_URL})" in payload.description


def test__when_merge_commit__should_link_subtitle_to_pr() -> None:
    link = ci_discord_message.subtitle_link(MERGE_MSG, SHA, REPO)

    assert link.startswith("[Merge pull request #204")
    assert "github.com/foxpoint-se/eel/pull/204" in link


def test__when_squash_commit__should_link_subtitle_to_pr_from_hash_suffix() -> None:
    message = "fix: fetch tags before checking out a release (#205)"
    link = ci_discord_message.subtitle_link(message, SHA, REPO)

    assert "pull/205" in link
    assert "fix: fetch tags" in link


def test__when_release_notes_long__should_truncate() -> None:
    notes = "x" * 600
    truncated = ci_discord_message.truncate(notes, ci_discord_message.NOTES_MAX_LEN)

    assert truncated.endswith("…")
    assert len(truncated) == ci_discord_message.NOTES_MAX_LEN


@pytest.mark.parametrize(
    ("message", "expected_pr"),
    [
        (MERGE_MSG, "204"),
        ("fix: something (#99)", "99"),
    ],
)
def test__when_message_has_pr_number__should_extract_it(
    message: str,
    expected_pr: str,
) -> None:
    assert ci_discord_message.pr_number_from_message(message) == expected_pr
