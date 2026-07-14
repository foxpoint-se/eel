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
- [ ] **`source_me.sh` docs** — explain why ROS + venv + workspace must be sourced before `ros2 run`
- [ ] **`source_me.sh` PATH vs activate** — explore `export PATH=".venv/bin:$PATH"` instead of `source venv/bin/activate`; discuss later
- [ ] **`make setup`** — discuss whether it should wrap `install` only, or `install` + `build` too
- [ ] **Makefile ↔ Dockerfile parity** — explore running same make targets in Docker/CI vs separate pip/rosdep steps
  - Docker may still need `--break-system-packages` / no venv; goal is no silent divergence
- [ ] **wget deps (ms5837, pi_ina226)** — discuss moving to proper pip/git deps or a dedicated install script
- [ ] **rosdep** — document how it fits: `package.xml` → apt; what rosdep can't cover and how we handle that
  - Optional: custom rosdep rules for pip-only packages
- [ ] **uv (optional)** — nicer dev pip/venv UX only; not a ROS replacement; defer unless we want it
- [ ] **No apt / rosdistro release** — explicitly out of scope
- [ ] **Agent rules** — add Cursor rules (and Claude/Copilot etc. if useful): project board location, conventional commits (value-focused), and other conventions — discuss when we get here
- [ ] **Remove this file before merge**
