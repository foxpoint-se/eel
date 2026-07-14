# Issue #128 — packaging & install cleanup

Track: https://github.com/foxpoint-se/eel/issues/128

Work from a branch off `main`; rebase `feat/tanks-pid-tuning` / other branches after merges land here.

- [x] **Branch setup** — create feature branch from `main` for this work
- [x] **`scripts/COLCON_IGNORE`** — keep dev scripts out of colcon (was on tank branch, needs re-adding here)
- [x] **README: dev vs Docker** — two clear install paths in getting started (small change only)
- [ ] **README full cleanup** — separate issue: obsolete content, rephrase, hardware notes audit
- [ ] **Python deps declaration** — decide lean standard: `setup.py` `install_requires` vs `pyproject.toml` vs `requirements.txt`
  - Prefer package-manifest style if fully supported by ament/colcon; not a fan of standalone `requirements.txt`
- [ ] **`pyproject.toml` feasibility** — check ROS2/ament_python constraints; compare with nanomodem (`pyproject.toml` + hatchling, no ROS)
- [ ] **Pin pip dependencies** — once format is chosen, lock versions for reproducible installs
- [ ] **`source_me.sh` docs** — explain why ROS + venv + workspace must be sourced before `ros2 run`
- [ ] **`make setup`** — discuss whether it should wrap `install` only, or `install` + `build` too
- [ ] **Makefile ↔ Dockerfile parity** — explore running same make targets in Docker/CI vs separate pip/rosdep steps
  - Docker may still need `--break-system-packages` / no venv; goal is no silent divergence
- [ ] **wget deps (ms5837, pi_ina226)** — discuss moving to proper pip/git deps or a dedicated install script
- [ ] **rosdep** — document how it fits: `package.xml` → apt; what rosdep can't cover and how we handle that
  - Optional: custom rosdep rules for pip-only packages
- [ ] **uv (optional)** — nicer dev pip/venv UX only; not a ROS replacement; defer unless we want it
- [ ] **No apt / rosdistro release** — explicitly out of scope
- [ ] **Remove this file before merge**
