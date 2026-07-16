# Issue #130 — README audit

Track: https://github.com/foxpoint-se/eel/issues/130

Not for merge. Remove before PR merge.

Every section must be decided (**keep / rephrase / move / delete**) and implemented before merge — no “optional” leftovers.

**Principle:** README = how to run *today* (short, current). Long / maybe-stale but still useful → **`eel/docs/`**. ~5 lines and still true → README; longer archive → `docs/`. Link existing promotion pages when they already cover a topic. Before merge: file GH issue for sync `eel/docs/` ↔ promotion site; leftover VL53/ADS scripts tracked in **#137**.

- [x] **Title / preface** — rephrase: one line under `# Eel`: ROS 2 nodes and bringup for the AUV (simulation and production).
- [x] **Prerequisites** — rephrase: three Ubuntu+ROS pairs; point at Docker as alternative (Getting started).
- [x] **Agent rules** — `AGENTS.md` + Cursor: prefer `eel/docs/` for long notes; linking existing promotion articles OK.
- [x] **Getting started → Local development** — keep: tighten prose; fix clone placeholder; note `make setup` only after sourcing.
- [x] **Getting started → Docker** — keep: fix `docker/` path wording; mention humble/jazzy/lyrical briefly; no compose walkthrough.
- [x] **Enable SPI access on Ubuntu** — rephrase: short why+run (tank fill / MCP3208 SPI → `make spidev-permissions`, reboot). Delete inline udev block. Fix script `$SUDO_USER`/`$USER`.
- [x] **I2C addresses** — trim to IMU `28`, battery `40`. Drop VL53 / ADS pots / pressure rows. Leftover scripts/deps → **#137**.
- [x] **Notes on running `imu` node** — rephrase: `make install-i2c`, reboot, `ros2 run eel imu`. Drop link/smoke test. Fix `install-i2c` user like SPI.
- [x] **Notes on running `gnss` node** — lean README: ports Ålen `/dev/ttyUSB1`, Tvålen `/dev/ttyUSB0`, `serial_port`, `dialout`. Drop Medium/smoke/“can’t remember”. **Move** UART/`serial-getty`/`config.txt` block → `docs/` (note: may be obsolete for USB GPS).
- [x] **Notes on running `motor` node** — keep in README (light rephrase): `rpi.gpio-common` + `dialout` + reboot.
- [x] **Notes on running `rudder` node** — keep in README (light rephrase): `pigpiod` / `make start-pigpio`.
- [x] **Notes on networking when using ethernet cable** — delete YAML from README. Add one-line link to promotion *How to use the UI and Eel together*. WireGuard unused → out of README (no move).
- [x] **Installing modem** — rephrase short README: Pi needs 4G → `make install-modem` + Sixfab link. Drop manual quectel/symlink prose. Fix `/urs/bin` → `/usr/bin` in `scripts/modem/install_modem_software.sh`.
- [x] **Scripts and service files for auto modem/VPN** — **Delete** all OpenVPN prose/scripts from README. Modem auto-connect: README points at `scripts/modem/install_modem_service.sh` / `make install-modem`. **Move** wifi-fallback-then-modem narrative → `docs/` (not in current script; preserve idea).
- [ ] **Implement pass** — (1) create `docs/` + `docs/README.md` index + move archived text; (2) rewrite README (preface, getting started, **Hardware bring-up** with subheadings for SPI/I2C/IMU/GNSS/motor/rudder/modem, link ethernet→promotion); (3) script fixes (`$SUDO_USER`/`$USER`, `/urs/bin`); (4) file GH issue for docs↔promotion sync; (5) remove this checklist; (6) open PR.
