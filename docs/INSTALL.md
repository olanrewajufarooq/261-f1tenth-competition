# Installation guide

These instructions target the native ROS 2 Lyrical adaptation of the V1.1
competition starter workspace. This port is experimental until native build
and simulator checks are recorded in [LYRICAL_VALIDATION.md](LYRICAL_VALIDATION.md).
Use a fresh terminal without another ROS distribution or workspace sourced.

## 1. Target system

- Ubuntu 26.04 LTS, x86_64
- ROS 2 Lyrical
- Bash shell
- Native installation, not Docker

Check the machine:

```bash
lsb_release -a
uname -m
locale
```

A UTF-8 locale is required. An empty `LC_ALL` is normal when categories inherit
from `LANG`.

## 2. Install all system prerequisites

Install the Ubuntu packages required to add the ROS repository, clone and build
the workspace, and create the Python virtual environment:

```bash
sudo apt update
sudo apt install -y \
  locales \
  software-properties-common \
  curl \
  ca-certificates \
  git \
  build-essential \
  pkg-config \
  python3 \
  python3-dev \
  python3-pip \
  python3-venv
```

Configure a UTF-8 locale if the machine does not already use one:

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

`LANG=en_US.UTF-8` is expected. An empty `LC_ALL` is also valid when locale
categories inherit from `LANG`.

The system Python is expected to be Python 3.14. Do not create this workspace's
virtual environment using a separate Python 3.12 installation: the ROS binary
Python modules must match the system interpreter.

## 3. Install ROS 2 Lyrical

Enable Universe and install the ROS apt-source helper:

```bash
sudo add-apt-repository -y universe
sudo apt update

ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')

curl -fL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

Install ROS 2 Desktop and its development tools:

```bash
sudo apt install -y ros-lyrical-desktop ros-dev-tools qt6-base-dev
```

Load and verify ROS:

```bash
source /opt/ros/lyrical/setup.bash
echo "$ROS_DISTRO"
ros2 pkg prefix rviz2
```

Expected:

```text
lyrical
/opt/ros/lyrical
```

Initialize rosdep once:

```bash
sudo rosdep init
rosdep update
```

If rosdep reports that `20-default.list` already exists, continue with
`rosdep update`.

At this point, the manually installed prerequisites are complete. During
workspace setup, `rosdep` installs package-level dependencies such as
Ackermann messages, Nav2 map server, robot state publisher, RViz, and
the Foxglove bridge. The setup script also creates the virtual environment and
installs the pinned Python simulator dependencies inside it.

## 4. Verify prerequisites

```bash
git --version
/usr/bin/python3 --version
python3 -m venv --help >/dev/null && echo "python3-venv: OK"
rosdep --version
colcon --help >/dev/null && echo "colcon: OK"
test -f /opt/ros/lyrical/setup.bash && echo "ROS 2 Lyrical: OK"
```

If all commands finish without an error, continue with the repository setup.

## 5. Clone this repository

```bash
git clone --branch lyrical --single-branch \
  https://github.com/olanrewajufarooq/261-f1tenth-competition.git
cd 261-f1tenth-competition
```

Confirm that the Lyrical adaptation branch is active:

```bash
git branch --show-current
```

Expected output:

```text
lyrical
```

## 6. Set up and build

```bash
./scripts/setup_workspace.sh
```

The script:

1. clones the upstream `dev-jazzy` ROS bridge at the pinned commit;
2. clones the JAX simulator backend at the pinned commit;
3. creates `starter_ws/.venv` with access to system ROS Python packages;
4. retains the baseline setuptools 79.0.1 compatibility pin;
5. installs the simulator backend and Python compatibility fixes;
6. resolves ROS dependencies for Lyrical with rosdep (may prompt for sudo);
7. builds the C++20 / Qt6 dashboard and workspace through the virtual-environment Python interpreter;
8. verifies ROS/simulator imports, performs a small JAX computation, and runs pip check.

The `.platform` marker records `lyrical:<system Python major.minor>`. Setup writes
it before colcon builds so an interrupted build can resume. A marker alone does
not mean the build succeeded. Launch commands also require the built workspace
and virtual environment. Never reuse generated files from a Jazzy checkout.

## 7. Launch

Simulator:

```bash
./scripts/launch_sim.sh
```

RViz opens automatically with the competition simulator and shows a fixed
F1-style race dashboard. It displays status, current lap, completed laps, last
time, best time, and recent lap history. The first finish-line crossing is an
out-lap; it starts the live stopwatch, and the next crossing completes lap 1.
The Foxglove bridge also runs in the background and remains available at
`ws://localhost:8765`, but the browser is not opened automatically.

PS5 teleoperation, in another terminal:

```bash
./scripts/launch_ps5_teleop.sh
```

Hold L1 to enable the car, steer with the left stick, use R2 for forward
throttle, L2 for reverse, and Cross to stop. Releasing L1 stops the car. Do not
run teleoperation and a team controller together because both publish to `/drive`.

## 8. Validate topics

With the simulator running:

```bash
./scripts/smoke_test.sh
```

Manual checks:

```bash
ros2 topic info /scan
ros2 topic info /ego_racecar/odom
ros2 topic info /drive
```

Expected:

- `/scan`: one publisher;
- `/ego_racecar/odom`: one publisher;
- `/drive`: one simulator subscriber;
- when a controller runs, `/drive` also has one publisher.

## 9. Validate lap timing

The simulator publishes authoritative completed-lap data. The competition race
monitor adds a live stopwatch, best time, and session history:

```bash
ros2 topic echo /competition/race_status
ros2 topic echo /competition/current_lap_time
ros2 topic echo /competition/lap_count
ros2 topic echo /competition/last_lap_time
ros2 topic echo /competition/best_lap_time
ros2 topic echo /competition/lap_history
ros2 topic echo /competition/collision_count
```

Expected sequence:

1. `OUT_LAP` after startup;
2. `RACING` after the first finish-line crossing;
3. `/competition/lap_count` becomes `1` after the next crossing;
4. last, best, and history receive the simulator's completed-lap time.

Using RViz **2D Pose Estimate** resets the timing session along with the car.
It also resets the session collision counter. A continuous contact with a wall
counts as one collision; leaving and hitting something again counts another.

## 10. RViz

The normal simulator launch starts RViz and the **Race Dashboard** panel
automatically. The dashboard is an RViz plugin in the `competition_ui` package
and subscribes to the `/competition/*` timing topics.

To restart RViz manually while the simulator is still running:

```bash
source /opt/ros/lyrical/setup.bash
source starter_ws/.venv/bin/activate
source starter_ws/install/setup.bash

rviz2 -d starter_ws/src/competition_bringup/config/competition.rviz
```

A message containing the following can appear when the map first renders:

```text
active samplers with a different type refer to the same texture image unit
```

It is non-blocking when the map, vehicle, and LiDAR remain visible and RViz
continues running.

## 11. Foxglove

Connect <https://app.foxglove.dev> to:

```text
ws://localhost:8765
```

Import the supplied layout:

```text
starter_ws/src/f1tenth_gym_ros/config/foxglove/gym_bridge_foxglove.json
```

## Troubleshooting

### Another ROS distribution is already sourced

If a script reports a conflicting ROS distribution or prefix, open a fresh
terminal and remove any automatic Jazzy/workspace sourcing from that terminal's
startup configuration. Merely unsetting `ROS_DISTRO` does not remove old Python
or library paths. Source only `/opt/ros/lyrical/setup.bash` for this branch.

### Existing workspace or virtual environment is incompatible

Setup refuses unmarked build outputs, a different platform marker, a different
Python minor version, a copied/moved venv, or a venv without access to system ROS packages. Preserve
old outputs by moving the following out of `starter_ws` into a separate backup
directory: `.venv`, `build`, `install`, `log`, and `.platform` (when present).
Then rerun setup in a fresh terminal. Keep simulator source checkouts and student
controller source files. Python venvs are not relocatable: recreate the venv at
the new worktree path instead of copying one from another checkout.

### Setup cannot resolve a Lyrical dependency

Do not install a Jazzy package as a substitute. Save the complete rosdep or pip
error and check the named package's Lyrical / Python 3.14 support. The pinned
simulator advertises Python 3.14 support, but this branch's full native package
combination still needs validation.

### Dashboard cannot load

The dashboard must compile against Qt6, matching Lyrical RViz on Ubuntu 26.04.
After setup, inspect missing or mixed Qt libraries with:

```bash
ldd starter_ws/install/competition_ui/lib/librace_dashboard_panel.so
```

There should be no `not found` entries or Qt5 libraries. Launch RViz and confirm
the Race Dashboard is visible and updates; a successful build alone does not
prove that the plugin loads.


### The bridge cannot import f1tenth_gym

Symptom:

```text
ModuleNotFoundError: No module named 'f1tenth_gym'
```

Rebuild using the virtual-environment interpreter:

```bash
cd starter_ws
source /opt/ros/lyrical/setup.bash
source .venv/bin/activate
python /usr/bin/colcon build --symlink-install
```

Verify the generated bridge shebang starts with the workspace virtual
environment path.

### Numba fails while importing coverage

Symptom:

```text
AttributeError: module 'coverage.types' has no attribute 'Tracer'
```

Repair:

```bash
source starter_ws/.venv/bin/activate
python -m pip install --upgrade 'coverage>=7.6'
python -m pip check
```

### setuptools conflict

```text
colcon-core requires setuptools<80
```

Repair:

```bash
source starter_ws/.venv/bin/activate
python -m pip install 'setuptools==79.0.1'
python -m pip check
```

### PyNaCl requires cffi

```bash
source starter_ws/.venv/bin/activate
python -m pip install --upgrade cffi
python -m pip check
```

