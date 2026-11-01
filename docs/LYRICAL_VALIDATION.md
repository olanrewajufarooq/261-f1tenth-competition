# Lyrical adaptation validation

Status: **experimental; native ROS build and runtime validation pending**.

## Source and environment

- Branch: `lyrical`, based on V1.1 commit `2c15feed39b325e13433a72a5ebf24d972cfcb12`.
- Target: native Ubuntu 26.04 x86_64 with ROS 2 Lyrical Luth, system Python 3.14,
  C++20, and Qt6.
- Inspection host: Ubuntu 26.04.1, x86_64, Python 3.14.4.
- ROS is not installed on the inspection host. Installing it requires the user's
  interactive sudo authentication; no ROS build or GUI run is claimed here.
- Bridge pin: `2c019810b4098ed82d44cad1d7d282a2dfc12421` (`dev-jazzy`).
- Simulator pin: `ec3b1a4693ed20317e42e57a9454d94b824cec81` (`dev-jax`).
- Exact pins remain unchanged; physics and competition topic contracts are not
  intentionally changed.

## Checks performed

- Shell syntax checks and Git whitespace checks passed during implementation.
- All 12 automated environment tests passed. They exercised a clean/Lyrical shell, conflicting ROS
  distributions and prefixes, stale/unmarked outputs, platform markers, copied/moved virtual environments,
  and virtual environments created before a Python minor-version upgrade.
- Setup on the inspection host exits with an actionable missing-Lyrical message
  before cloning dependencies or building.
- Official rosdistro metadata lists Lyrical releases for Ackermann messages,
  joystick drivers, Nav2, RViz, robot state publisher, xacro, and Foxglove bridge.
  This establishes release metadata, not successful installation on this host.
- The Qt6 development rosdep key resolves to Ubuntu's `qt6-base-dev`.
- The exact pinned simulator metadata requires Python >=3.12 and lists Python
  3.14; it uses PyQt6. The pinned bridge uses conventional ROS 2 Python APIs.

Run repository-only checks with:

```bash
python3 -B -m unittest discover -s tests -v
for script in scripts/*.sh; do bash -n "$script" || exit; done
git diff --check
```

A separate pip dry-run on Python 3.14 found candidate binary-wheel metadata
for the pinned simulator's direct and transitive dependencies, including JAX,
Numba, PyQt6, and the retained setuptools pin. The run was stopped during full
wheel downloads due to the download time. It did not complete installation or
produce a final resolver report; this is not a dependency-install success claim.

## Native checks to complete

Follow INSTALL.md in a fresh terminal, then record actual output and installed
versions here. Update this status only after the corresponding checks pass.

```bash
source /opt/ros/lyrical/setup.bash
./scripts/setup_workspace.sh
source starter_ws/.venv/bin/activate
source starter_ws/install/setup.bash
python --version
python -m pip check
python -m pip freeze
ros2 doctor --report
cd starter_ws
python /usr/bin/colcon test
python /usr/bin/colcon test-result --verbose
```

| Check | Status |
|---|---|
| Native apt installation / rosdep resolution | Pending |
| Full Python dependency install and pip check | Pending |
| JAX CPU computation | Pending |
| Complete colcon build / applicable tests | Pending |
| Qt6 plugin linking and RViz dashboard loading | Pending |
| Live scan, odometry, TF, map server, and drive interface | Pending |
| OUT_LAP → RACING → lap 1, last/best/history | Pending |
| Continuous/new collisions and pose reset | Pending |
| PS5 enable, stop, release, timeout, speed limits | Pending; physical controller required |
| Foxglove connection and supplied layout | Pending |
| Shutdown/relaunch and repeat setup | Pending |

Launch in separate terminals from the repository root:

```bash
./scripts/launch_sim.sh
./scripts/launch_ps5_teleop.sh
timeout 120s ./scripts/smoke_test.sh
./scripts/stop_car.sh
```

Use INSTALL.md's timing and collision checks. Do not run teleoperation together
with a student controller. Inspect the plugin with `ldd` and verify there are no
missing libraries or mixed Qt5 dependencies. Record any test failures by name,
including pre-existing failures; do not infer runtime success from topic names.

## References

- [Lyrical Ubuntu installation](https://docs.ros.org/en/lyrical/Get-Started/Installation/Ubuntu-Install-Debs.html)
- [Lyrical supported platforms](https://github.com/ros2/ros2_documentation/blob/lyrical/source/Releases/lyrical/supported-platforms.rst)
- [Pinned simulator requirements](https://github.com/f1tenth/f1tenth_gym/blob/ec3b1a4693ed20317e42e57a9454d94b824cec81/pyproject.toml)
- [Pinned bridge dependencies](https://github.com/f1tenth/f1tenth_gym_ros/blob/2c019810b4098ed82d44cad1d7d282a2dfc12421/package.xml)
