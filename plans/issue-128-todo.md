# Issue #128 — packaging & install cleanup

Track: https://github.com/foxpoint-se/eel/issues/128

Work from a branch off `main`; rebase `feat/tanks-pid-tuning` / other branches after merges land here.

- [x] **Branch setup** — create feature branch from `main` for this work
- [x] **`scripts/COLCON_IGNORE`** — keep dev scripts out of colcon (was on tank branch, needs re-adding here)
- [x] **README: dev vs Docker** — two clear install paths in getting started (small change only)
- [x] **README full cleanup** — #130
- [x] **Python deps declaration** — `setup.py` for pip, `package.xml` for ROS; no `requirements.txt`
- [x] **`pyproject.toml` feasibility** — defer; ament_python stays on `setup.py` for now
- [x] **Pin pip dependencies** — `>=` lower bounds in `setup.py`; verify CI on humble/jazzy/lyrical
- [x] **`source_me.sh` docs** — explain why ROS + venv + workspace must be sourced before `ros2 run`
- [x] **`source_me.sh` PATH vs activate** — keep `activate`; (venv) prompt is a useful reminder
- [x] **`make setup`** — `install` + `build` after sourcing; `setup`/`build`/`test` fail fast if ROS not sourced
- [x] **Makefile ↔ Dockerfile parity** — decided: keep separate (venv local vs system pip in Docker); shared via `setup.py` / `package.xml`
- [x] **wget deps (ms5837, pi_ina226)** — `pi_ina226` via `pip install git+…`; `ms5837` left to #132 (script-only)
- [x] **rosdep** — document how it fits: `package.xml` → apt; what rosdep can't cover and how we handle that
  - Optional: custom rosdep rules for pip-only packages
- [x] **uv (optional)** — deferred; nicety for pip/venv only, doesn't improve ROS sourcing/run path
- [x] **No apt / rosdistro release** — explicitly out of scope
- [x] **Agent rules** — spun out to #133
- [ ] **Remove this file before merge**
