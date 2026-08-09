# World sim — first iteration goals

**Temporary** — delete this file before merging the finished work to `main`.

Related: [#212](https://github.com/foxpoint-se/eel/issues/212)

## Target architecture (path 3)

Same idea as TurtleBot / Clearpath / Stonefish / Gazebo bringups:

1. **Plant** — owns fake-world physics (depth first).
2. **Device I/O** — chip drivers only (Bar02, BNO055, …). Started on the **boat**, not in full world-sim.
3. **Robot logic** — always runs (center depth, velocity, `PressureStatus`, nav, …).

In sim: **don’t start chip I/O**. Plant (or later Gazebo/Stonefish) supplies **raw/sensor-level** data. Logic keeps publishing eel topics (`pressure/status`, …).

**Not the goal:** every device node subscribes to plant topics and stuffs values in (path 1 — today’s `pressure_sim` mess).  
**Not the long-term story alone:** in-memory DI drivers in one process (path 2) — fine as an *implementation detail* for our small plant, but Gazebo/Stonefish are other processes that speak ROS topics / plugins.

**Plant edge:** prefer standard ROS msgs where practical (`sensor_msgs/FluidPressure`, later `Imu`, …), with a thin adapt into eel msgs. That prepares switching to Stonefish/Gazebo.

## Phase 1 — Plant + start pressure split (path 3)

**Goal:** Plant owns depth physics. Begin separating pressure **chip I/O** from **logic** so we don’t build the wrong habit. Boat/CI still work.

- [x] New in-repo package `eel_world_sim`
- [ ] Plant runs today’s depth math (lifted from pressure sim); math separate from ROS I/O
- [ ] Plant publishes **raw** depth (aim toward standard msg; not `pressure/status`)
- [ ] Start pressure SoC split: clear boundary between “get raw depth” (I/O / source) and “build `PressureStatus`” (logic). Hardware and stub sources; **no** dive physics in pressure.
- [ ] Stub/CI: pressure logic still runs with dumb raw depth (no plant, no hardware)
- [ ] Simple launch: plant alone; optional plant + pressure-logic path for sanity
- [ ] Sanity: plant depth moves with cmds; pressure logic can consume raw depth without `pressure_sim` physics

Out of Phase 1: full IMU/etc. splits, GUI, Stonefish/Gazebo, URDF, polished multi-sim bringups.

## Later phases (same iteration / follow-ups)

- **Finish path-3 bringup** — sim launch: plant + logic, no chip drivers. Sibling launches: real / our world / (later) Stonefish|Gazebo.
- **Delete dead pressure sim physics** — if anything remains after the split.
- **Minimal GUI** — depth + cmds on real cmd topics.
- **Contract written** — plant in/out; who publishes what.

## Non-goals (now)

Full mission stack, full IMU/GNSS/tanks plant, Stonefish/Gazebo integration, fancy GUI, new repo, URDF-driven physics, ros2_control.

## Later / discuss

- **URDF** — model the plant could *use* for dynamics (not RViz-only). Stonefish won’t take URDF as drop-in. Spin out an issue when relevant.
- **ros2_control** — for motors/actuators later? Helps a lot with **Gazebo**; Stonefish is topic-based natively, so less automatic win. Revisit: when/whether it helps eel (few actuators, Python-first, Stonefish-oriented) vs cost.

## Done when (first iteration)

- [ ] Plant owns depth physics; pressure has no cross-topic dive math
- [ ] Path 3 shape clear: chip I/O vs logic vs plant (even if only pressure is split)
- [ ] Sim can run plant + logic without relying on `pressure_sim` physics
- [ ] Stub/CI path still works without hardware or plant physics
- [ ] Written topic/msg contract at the plant edge
- [ ] Tiny GUI optional but useful
- [ ] This doc deleted before merge
- [ ] Interactive rebase (or similar) so the branch commits read cleanly for the changelog before merge
- [ ] Clean up redundant files left by the parallel path (e.g. old `pressure_sensor.py` / `pressure_sim` physics once the new I/O + logic + plant path is confirmed)
