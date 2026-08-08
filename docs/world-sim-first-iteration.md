# World sim — first iteration goals

**Temporary** — delete this file before merging the finished work to `main`.

Related: [#212](https://github.com/foxpoint-se/eel/issues/212)

## Idea

Same pattern as Stonefish/Gazebo: a **plant** owns the fake world (vehicle pose, crude dynamics). Sensor nodes stay ordinary and only read plant state. Later we can swap our plant for Stonefish behind the same kind of ROS edge.

## First mergeable step

Additive: run the world plant **as well**. Existing nodes keep their built-in sim; everything still works as today. No big-bang cutover.

## Goals

- **Launch** — New bringup launch for robot graph + plant. Grow it over time. Sibling launches later for real HW / our sim / other sims (same pattern). Not “one launch for everything” in this slice.
- **Depth only** — Plant holds vehicle state and the dive model (e.g. thrust + pitch → depth). Publishes depth; pressure path does not invent physics or subscribe to motor/IMU.
- **Pressure stub without physics** — `simulate:=true` (or equivalent) still runs off-Pi for CI / integration tests, with no cross-topic physics — dumb readings only. Plant is what makes depth move when you want a fake world.
- **Clean graph** — Plant is the busy node. Pressure no longer looks like it secretly owns the boat.
- **Move physics out of pressure** — Delete the cross-topic sim math from the pressure sim path (after the additive step above).
- **Topic contract** — Write down: plant subscribes to X, publishes Y; who publishes `pressure/status`. Pick one story and stick to it.
- **Standard msgs at the plant edge** — Prefer common ROS types where we can (e.g. `sensor_msgs/FluidPressure`, later `Imu` / `Odometry`), with a thin translation to eel topics/msgs (`pressure/status`, …). Either start that way early, or ship a simpler plant first and refactor to this pattern once it works — decide when we hit the contract.
- **Minimal GUI** — Show depth + driving cmds; controls publish the **real** cmd topics (not a private channel). Enough to demo without only reading logs.
- **One real consumer** — Something besides the GUI still sees depth (e.g. depth control / localization).
- **Package** — Clear package boundary in this repo (e.g. plant + GUI). Stay in eel for now; own repo later if needed.
- **Lightly reusable** — Think “water-world plant,” but don’t build a multi-robot framework in v1.

## Non-goals

Full mission stack, IMU/GNSS/tanks plant, Stonefish integration, fancy GUI, new repo, URDF-driven physics.

## Later / discuss

- **URDF** — A simple robot model (e.g. cylinder + mass) that the plant actually *uses* for dynamics — not RViz-only eye candy — and that could feed Gazebo/Stonefish later. Stonefish won’t take URDF as drop-in; parsing URDF into our crude plant is a real chunk of work. Park for now; likely spin out a separate issue when we revisit.

## Done when

- [ ] Sim launch entry exists and is clearly the place to grow
- [ ] World plant can run alongside existing stack without breaking normal sim
- [ ] Depth comes from plant when using the world; pressure sim has no physics (stub only for CI)
- [ ] Graph and publisher story match the written contract
- [ ] Tiny GUI: state + cmds on real topics
- [ ] At least one non-GUI consumer still works
- [ ] In-repo package boundary; no new repo
- [ ] This doc deleted before merge
