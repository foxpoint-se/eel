# Agent guidance (eel)

## Project board

Backlog: https://github.com/orgs/foxpoint-se/projects/1  
When creating a GitHub issue, add it to that board.

Also set **Priority** (org issue field), **Type** (Bug / Feature / Task), and **label(s)**. Inspect what already exists in the repo (labels, types, board fields) and pick what fits — don’t invent a parallel scheme.

## Commits

Use conventional commits. **The reader must understand why** the change was made — state the broken situation and who was affected, not what you changed in code.

Bad: `fix: publish drain-to-empty tank target in status` (code mechanics; no problem stated)  
Good: `fix: drain-to-empty showed no target on dashboard while pump was running`

Body (when needed): why it mattered — e.g. "Operators could not tell the boat was executing a drain command." Root cause and file names belong in the PR.

**Release types** (semantic-release on `main` — `feat`/`fix` cut a version; `chore`/`ci`/`docs` do not):

- **`feat:`** — new or changed **boat/runtime behavior** (ROS nodes, fleet-visible capability).
- **`fix:`** — **application bug** (wrong behavior on Pi/boat).
- **`chore:`** — tooling, CI plumbing, version sync metadata; small lint/format with no meaningful source diff.
- **`ci:`** / **`docs:`** — workflows or documentation only.

Prefer `chore:` / `ci:` for infra-only work that never touches application source (Discord notify, Docker tagging, workflow YAML).

**Bulk lint/format** across many source files: `feat:` or `fix:` is fine — we touch real code and a version bump is a useful marker even when behavior is unchanged.

Before committing, ask: **what changelog entry should this merge produce?** Match commit type and count to that — squash PR wiring into the meaningful commit.

## Before commit

```bash
source source_me.sh && make test
```

If lint fails: `make fix-lint`, then `make test` again.

## Scope

Stay on the asked task. Don’t expand the PR or invent unrelated work.  
Don’t create or rewrite markdown/docs unless asked.

## Notes / long-form findings

Prefer **`docs/` in this repo** for searchable write-ups and archived how-tos (keeps agents aware).  
The sibling **promotion** repo can publish those later (optional sync); linking existing promotion articles is fine. Don’t grow the eel README with field notes.

## Code style

Prefer short, clearly named functions — the call chain should read as the story.  
Don’t bury non-trivial logic in long bodies; extract into helpers, modules, or classes.  
Match the style of nearby code.

## Tests

Name: `test__when_<condition>__should_<outcome>` (double underscores).

Prefer **pure unit tests** — extract logic from ROS callbacks/nodes into small functions, then test those. Don’t grow 50-line tests full of mocks; refactor production code until each test is ~10 lines: setup (helpers ok), act, assert.

## Typing

Strict mypy runs on the file list in `pyproject.toml` (`make typecheck`, part of `make test`). Grow that list as files are clean.

**External imports:** incomplete third-party stubs are fine (`ignore_missing_imports`). Prefer **`types-*`** packages on PyPI when they exist (e.g. `types-pyserial`).

**Our boundaries:** type what we control even when the library doesn’t — callback args (`msg: PressureStatus`), returns, params. If a foreign API stays opaque but we use it often, add a local stub under `typings/` or a thin typed wrapper.

**Avoid** `Any`, `cast(...)`, and `# type: ignore` unless they are genuinely the cleanest fix — prefer proper types, generics (`ParamSpec` / `TypeVar`), or fixing stubs first.
