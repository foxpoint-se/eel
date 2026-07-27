#!/usr/bin/env python3
"""Build Discord notify payload for main-branch CI workflow."""

from __future__ import annotations

import os
import re
import sys
from dataclasses import dataclass

COLOR_FAIL = "0xED4245"
COLOR_SUCCESS = "0x57F287"
DISTROS_NOTE = "Checks + docker: humble, jazzy, lyrical"
NOTES_MAX_LEN = 500
SUBTITLE_MAX_LEN = 120

_MERGE_PR = re.compile(r"Merge pull request #(\d+)", re.IGNORECASE)
_SQUASH_PR = re.compile(r"\(#(\d+)\)\s*$")

_JOB_LABELS = {
    "test-checks": "checks",
    "build": "docker build",
    "release": "release",
}


@dataclass(frozen=True)
class DiscordPayload:
    title: str
    description: str
    color: str
    url: str
    status: str


def truncate(text: str, max_len: int) -> str:
    stripped = text.strip()
    if len(stripped) <= max_len:
        return stripped
    return stripped[: max_len - 1].rstrip() + "…"


def first_line(message: str) -> str:
    return message.splitlines()[0].strip() if message.strip() else ""


def pr_number_from_message(message: str) -> str | None:
    merge = _MERGE_PR.search(message)
    if merge:
        return merge.group(1)
    squash = _SQUASH_PR.search(first_line(message))
    if squash:
        return squash.group(1)
    return None


def subtitle_link(message: str, sha: str, repo: str) -> str:
    line = truncate(first_line(message), SUBTITLE_MAX_LEN)
    if not line:
        line = sha[:7]

    pr_number = pr_number_from_message(message)
    if pr_number:
        url = f"https://github.com/{repo}/pull/{pr_number}"
    else:
        url = f"https://github.com/{repo}/commit/{sha}"

    return f"[{line}]({url})"


def failed_jobs(
    test_checks: str,
    build: str,
    release: str,
) -> list[str]:
    results = {
        "test-checks": test_checks,
        "build": build,
        "release": release,
    }
    failed: list[str] = []
    for job, result in results.items():
        if result == "failure":
            failed.append(_JOB_LABELS[job])
    return failed


def is_success(
    test_checks: str,
    build: str,
    release: str,
) -> bool:
    if test_checks != "success" or build != "success":
        return False
    return release in {"success", "skipped"}


def build_discord_payload(
    *,
    test_checks: str,
    build: str,
    release: str,
    released: bool,
    version: str,
    release_notes: str,
    release_link: str,
    run_url: str,
    commit_sha: str,
    commit_message: str,
    repo: str,
) -> DiscordPayload:
    subtitle = subtitle_link(commit_message, commit_sha, repo)

    if not is_success(test_checks, build, release):
        failed = failed_jobs(test_checks, build, release)
        failed_text = ", ".join(failed) if failed else "unknown"
        body = f"{subtitle}\n\nFailed: {failed_text}\n{DISTROS_NOTE}"
        return DiscordPayload(
            title="Eel CI failed",
            description=body,
            color=COLOR_FAIL,
            url=run_url,
            status="Failure",
        )

    if released:
        title = f"Released eel v{version}"
        notes = truncate(release_notes, NOTES_MAX_LEN)
        parts = [subtitle, ""]
        if notes:
            parts.append(notes)
            parts.append("")
        parts.extend([DISTROS_NOTE, f"[Full release notes →]({release_link})"])
        return DiscordPayload(
            title=title,
            description="\n".join(parts),
            color=COLOR_SUCCESS,
            url=release_link or run_url,
            status="Success",
        )

    version_note = f" (still v{version})" if version else ""
    body = (
        f"{subtitle}\n\n"
        f"Build succeeded{version_note}\n"
        f"{DISTROS_NOTE}"
    )
    return DiscordPayload(
        title="Eel CI — no new version",
        description=body,
        color=COLOR_SUCCESS,
        url=run_url,
        status="Success",
    )


def _write_github_output(payload: DiscordPayload) -> None:
    output_path = os.environ.get("GITHUB_OUTPUT")
    if not output_path:
        return
    fields = {
        "title": payload.title,
        "description": payload.description,
        "color": payload.color,
        "url": payload.url,
        "status": payload.status,
    }
    with open(output_path, "a", encoding="utf-8") as handle:
        for key, value in fields.items():
            handle.write(f"{key}<<EOF\n{value}\nEOF\n")


def main() -> int:
    released = os.environ.get("RELEASED", "false").lower() == "true"
    payload = build_discord_payload(
        test_checks=os.environ.get("TEST_CHECKS_RESULT", ""),
        build=os.environ.get("BUILD_RESULT", ""),
        release=os.environ.get("RELEASE_RESULT", ""),
        released=released,
        version=os.environ.get("VERSION", ""),
        release_notes=os.environ.get("RELEASE_NOTES", ""),
        release_link=os.environ.get("RELEASE_LINK", ""),
        run_url=os.environ.get("RUN_URL", ""),
        commit_sha=os.environ.get("COMMIT_SHA", ""),
        commit_message=os.environ.get("COMMIT_MESSAGE", ""),
        repo=os.environ.get("GITHUB_REPOSITORY", ""),
    )

    print("Discord notify payload:")
    print(f"  title: {payload.title}")
    print(f"  status: {payload.status}")
    print(f"  url: {payload.url}")
    print(f"  description:\n{payload.description}")

    _write_github_output(payload)
    return 0


if __name__ == "__main__":
    sys.exit(main())
