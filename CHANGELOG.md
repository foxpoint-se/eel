# CHANGELOG

<!-- version list -->

## v1.2.10 (2026-08-03)

### Bug Fixes

- Cancelled dive still reported success after surfacing
  ([`8fd99af`](https://github.com/foxpoint-se/eel/commit/8fd99afcd2c58b6cd2d7af5747a4c2965f753a4d))

- LED pin stayed claimed after node restart
  ([`faa2e03`](https://github.com/foxpoint-se/eel/commit/faa2e03854e7d71151aca0ba45573af7a5f35ca4))

- Modem node crashed when registration or signal was zero
  ([`d94e752`](https://github.com/foxpoint-se/eel/commit/d94e7520b711eeaf720308481fc83579dacccc2c))

- Overlapping navigate goals were accepted but never ran
  ([`ef8704e`](https://github.com/foxpoint-se/eel/commit/ef8704e66b93680d171d04b2a2a1a12a64f0ac75))

- Tank pump could run forever when level never reached target
  ([`c615108`](https://github.com/foxpoint-se/eel/commit/c6151082e3a4bc35d66dbd785e79b773a783bf40))

### Chores

- Nav client half-wired leak abort looked like unfinished work
  ([`73ee797`](https://github.com/foxpoint-se/eel/commit/73ee797af1071560e38331aab9a0ae94e5dffb89))


## v1.2.9 (2026-08-02)

### Bug Fixes

- GNSS parse errors were swallowed with a broad except
  ([`c654957`](https://github.com/foxpoint-se/eel/commit/c6549576a8d014c648fb7c018200e2ed38329920))

### Chores

- Leftover VL53/ADS scripts implied tanks still used retired hardware
  ([`f159188`](https://github.com/foxpoint-se/eel/commit/f1591889d8096f86b1206380ecd0cd1e03e2678b))

- Package metadata listed personal maintainers instead of the förening
  ([`321791d`](https://github.com/foxpoint-se/eel/commit/321791d2b8b89614e82f889dd1ddeeb8a522dc68))

- Package metadata still said TODO after license was added
  ([`e36e495`](https://github.com/foxpoint-se/eel/commit/e36e4954ccad61ee3422773940cea05b072187d9))

- Repo had no license so reuse rights were unclear
  ([`b178675`](https://github.com/foxpoint-se/eel/commit/b17867545324e5f63632af6850bacbd3e81f2090))


## v1.2.8 (2026-08-02)

### Bug Fixes

- GNSS could publish lat/lon from different fixes
  ([`e8b23f0`](https://github.com/foxpoint-se/eel/commit/e8b23f07b2a9747ff00daad904ef635b677c6fc3))


## v1.2.7 (2026-08-02)

### Bug Fixes

- Drain-to-empty showed no target on dashboard while pump was running
  ([`b5e5d3b`](https://github.com/foxpoint-se/eel/commit/b5e5d3b3428ac416ae0e4ab96ec28fac3d0f8a4e))

### Chores

- Agents wrote commit messages that hid why a change was made
  ([`2fa0a29`](https://github.com/foxpoint-se/eel/commit/2fa0a2963622a5bbb8eab619037e0351072826b2))


## v1.2.6 (2026-08-01)

### Bug Fixes

- Abort navigate goal when pose is missing at execute
  ([`00cc839`](https://github.com/foxpoint-se/eel/commit/00cc839c24a85b5cd59804364ab2c404b93a3527))

- Set navigate result.success false on cancel
  ([`f3e4e33`](https://github.com/foxpoint-se/eel/commit/f3e4e338e9155f57bc5cb14850ba6595b4a247ef))


## v1.2.5 (2026-08-01)

### Bug Fixes

- Reject non-finite and negative dive goal values
  ([`83a2a3a`](https://github.com/foxpoint-se/eel/commit/83a2a3a923bd0f4ee407ccc86f590884c979238b))

- Validate incoming dive goal time and depth
  ([`b85ffc0`](https://github.com/foxpoint-se/eel/commit/b85ffc0fc75df4f284f7eb616b5a41e75d99ba8b))


## v1.2.4 (2026-07-31)

### Bug Fixes

- Skip IMU publish when euler read is missing
  ([`09eeb25`](https://github.com/foxpoint-se/eel/commit/09eeb25714f62137e0a8cefa449119801ed4e303))


## v1.2.3 (2026-07-30)

### Bug Fixes

- Localization and nav can get a GNSS fix off-Pi
  ([`abcbed3`](https://github.com/foxpoint-se/eel/commit/abcbed3dd65f7ebf52136738b98b09f2b9acdebc))

- Motor hardware driver stays import-safe off the Pi
  ([`0107b57`](https://github.com/foxpoint-se/eel/commit/0107b57bcf054b899deae14d3c2cb75750a5bc97))

- Mqtt_bridge sim logs cloud traffic instead of calling AWS
  ([`5e311b2`](https://github.com/foxpoint-se/eel/commit/5e311b295dc623d330573b9b55f6ba17d9e10ea0))

- Off-Pi sim stacks can run the LED node
  ([`73dbcb3`](https://github.com/foxpoint-se/eel/commit/73dbcb318fa85ba39ba26a3726d27be7c46daebc))


## v1.2.2 (2026-07-29)

### Bug Fixes

- Always destroy node even when cleanup fails
  ([`5973808`](https://github.com/foxpoint-se/eel/commit/5973808e01c8c43de7a95bd7339b1e3c9aa09409))

- Avoid rosout errors during hardware node shutdown
  ([`7e8b2c7`](https://github.com/foxpoint-se/eel/commit/7e8b2c7b076065f9a7c75afafbc81ddb00099197))

- Use try_shutdown in dive_demo goal callback
  ([`30660e6`](https://github.com/foxpoint-se/eel/commit/30660e6d7f99eceb5a11b8f0ea4c74453c14934c))

### Refactoring

- Co-locate integration boot test fixtures with test code
  ([`5d6bab7`](https://github.com/foxpoint-se/eel/commit/5d6bab724640484f4b5930a571bfdfe782a7c60a))

- Extract spin_node_until_shutdown helper
  ([`c57ccf0`](https://github.com/foxpoint-se/eel/commit/c57ccf0966f0427bd9824e4925663a524198a700))


## v1.2.1 (2026-07-28)

### Bug Fixes

- Let navigate and dive nodes import on humble and jazzy
  ([`855a0ba`](https://github.com/foxpoint-se/eel/commit/855a0baef95bfd545d7dede042e700739555d206))

- Reject tank nodes that launch without calibration params
  ([`e523388`](https://github.com/foxpoint-se/eel/commit/e5233882d096b2ca2bb3effb12cea5bbf87bd478))

- Stop tank nodes dying when launch omits param defaults
  ([`f45a331`](https://github.com/foxpoint-se/eel/commit/f45a331939310c6571d640ebb83a930e77473d8d))

### Documentation

- Remind agents to commit for the merged changelog story
  ([`0525604`](https://github.com/foxpoint-se/eel/commit/05256049a8b5b6477ff5963a488028d70ee64d09))

### Refactoring

- Reuse eel param and topic constants in sim startup launch
  ([`04cc853`](https://github.com/foxpoint-se/eel/commit/04cc8537327706397b6f7bc3a0bec0c5182ee743))

### Testing

- Catch sim-stack startup failures before they reach the boat
  ([`34feb82`](https://github.com/foxpoint-se/eel/commit/34feb8246299b9a5f5d2ab2bbeea902ee0dfbb77))

- Register launch_test marker in workspace pyproject.toml
  ([`c776a18`](https://github.com/foxpoint-se/eel/commit/c776a18ad374b228c9e2a9aa599cbaf5c265fc14))

- Stabilize colcon pytest runs for the startup test
  ([`7a924e4`](https://github.com/foxpoint-se/eel/commit/7a924e4160e2d4175b272e7585e19d499d43c1f0))

- Wait for all startup status topics in one pass
  ([`d7ee3b2`](https://github.com/foxpoint-se/eel/commit/d7ee3b29be5374ff304addc8de5759f6b1436b24))


## v1.2.0 (2026-07-28)

### Bug Fixes

- Apply ruff lint and format across codebase
  ([`f22893a`](https://github.com/foxpoint-se/eel/commit/f22893a2d35acb5a78a89cdb98cc21f5ad6cd9ba))

- Extend ruff lint scope to colcon package tests
  ([`b61f417`](https://github.com/foxpoint-se/eel/commit/b61f4173e64caead78df677455e5e2c37811044f))

- Restore buoyancy call in pressure sim after lint cleanup
  ([`7dd8b92`](https://github.com/foxpoint-se/eel/commit/7dd8b92c69f5899d9c6363a16e4fa322ae1b45eb))

### Documentation

- Document ruff lint commands in README
  ([`33abb77`](https://github.com/foxpoint-se/eel/commit/33abb77dab18f9ce6504b6ac79f4beb271939141))

- Encode test workflow and commit type guidance for agents
  ([`73c2589`](https://github.com/foxpoint-se/eel/commit/73c2589265acab33682b230618e4e5b586a33002))

### Features

- Add ruff lint and format checks to test-checks
  ([`4a324a9`](https://github.com/foxpoint-se/eel/commit/4a324a9b974deaccf22ed1ab6b70b1869d75276d))


## v1.1.1 (2026-07-27)

### Bug Fixes

- Preserve section order for unrecognized changelog headings
  ([`95f1119`](https://github.com/foxpoint-se/eel/commit/95f111949625528197d14edbd7a34fb0437807e1))

- Show features first in Discord release summary
  ([`750e518`](https://github.com/foxpoint-se/eel/commit/750e51810d707fa7fe8a18a9d849295aa8c831ea))


## v1.1.0 (2026-07-27)

### Bug Fixes

- Harden Discord notify for edge cases
  ([`a524bae`](https://github.com/foxpoint-se/eel/commit/a524bae2cd0dacec441b5871c0dfd5cce93db246))

- Sanitize Discord mentions and escape link labels
  ([`c82d253`](https://github.com/foxpoint-se/eel/commit/c82d25395ac38d95e6ff3bd908c8c611269bbb03))

- Use lowercase status for Discord action input
  ([`de71305`](https://github.com/foxpoint-se/eel/commit/de713056cbb21e811f08426b4752947b7ad27f34))

### Features

- Improve Discord CI notify for releases
  ([`8082e3a`](https://github.com/foxpoint-se/eel/commit/8082e3a3bd38b49e1f45c18c9c54c617bbd39f32))


## v1.0.1 (2026-07-27)

### Bug Fixes

- Fetch tags before checking out a release
  ([`739d275`](https://github.com/foxpoint-se/eel/commit/739d2755087c79e2aa049a952761abf53c12a954))

### Chores

- Split version sync note in README
  ([`6161e61`](https://github.com/foxpoint-se/eel/commit/6161e61b1043b5c5fbff66d3a5a52e79e05db745))


## v1.0.0 (2026-07-27)

- Initial Release
