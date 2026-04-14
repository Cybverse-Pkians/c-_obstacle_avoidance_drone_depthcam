# 🚁 Autonomous Obstacle Avoidance Drone — Depth Camera

> A C++ / ROS 2 system for autonomous drone obstacle avoidance using a depth camera, validated entirely in simulation with **ArduPilot SITL** and **Gazebo**.

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
![ROS2](https://img.shields.io/badge/ROS2-Humble-brightgreen)
![C++](https://img.shields.io/badge/language-C%2B%2B17-orange)
![Simulation](https://img.shields.io/badge/sim-Gazebo%20%7C%20ArduPilot%20SITL-9cf)

---

## 📖 Overview

This project implements a fully autonomous **obstacle avoidance pipeline** for a simulated drone. A depth camera streams distance data into a ROS 2 node written in C++. The node processes the point-cloud / depth image, detects obstacles in the drone's flight path, and sends corrective velocity or attitude commands back to the flight controller via **MAVROS** — all without any human in the loop.

The entire system runs in a **software-only environment**: ArduPilot SITL emulates the flight controller hardware, and Gazebo provides the physics simulation and sensor rendering. No real drone is needed to develop or test.

### Key Features

- Real-time depth image processing in C++ for low latency
- MAVROS bridge between ROS 2 and ArduPilot SITL over MAVLink
- Configurable obstacle detection threshold and avoidance manoeuvre parameters
- Fully simulated — safe to crash, iterate, and experiment
- Gazebo world with placed obstacles for repeatable testing

---

## 🏗️ System Architecture

```
┌──────────────────────────────────────────────────────────┐
│                      Gazebo Simulation                    │
│                                                          │
│  ┌─────────────┐    depth image    ┌──────────────────┐  │
│  │ Depth Camera│──────────────────▶│  ROS 2 Node      │  │
│  │  (plugin)   │                   │  (C++ avoidance) │  │
│  └─────────────┘                   └────────┬─────────┘  │
│                                             │ cmd_vel /  │
│  ┌──────────────────┐   MAVLink            │ set_mode   │
│  │  ArduPilot SITL  │◀────────────────────▶│  MAVROS    │
│  │  (ArduCopter)    │      UDP             └────────────┘  │
│  └──────────────────┘                                    │
└──────────────────────────────────────────────────────────┘
```

---

## 🔧 Prerequisites

| Requirement | Version | Notes |
|---|---|---|
| Ubuntu | 22.04 LTS | Required for ROS 2 Humble |
| ROS 2 | Humble Hawksbill | Desktop-full recommended |
| Gazebo | Harmonic (gz-sim 8) | or Garden (gz-sim 7) |
| ArduPilot | Latest `master` | Built from source |
| MAVROS | ROS 2 package | `ros-humble-mavros` |
| Python | 3.10+ | For ArduPilot scripts |
| CMake | 3.16+ | Build system |

> **GPU note:** Gazebo with depth camera is CPU-intensive. A dedicated GPU or at minimum 8 GB RAM is strongly recommended.

---

## ⚡ Quick-Start Guide

Follow these steps in order. Each section assumes you are working on a fresh **Ubuntu 22.04** installation.

### 1 — Install ROS 2 Humble

```bash
# Add ROS 2 apt repository
sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install ros-humble-desktop-full python3-colcon-common-extensions -y

# Source in every new terminal (add to ~/.bashrc to make permanent)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2 — Install MAVROS

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras -y
# Install GeographicLib datasets (required by MAVROS)
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

### 3 — Install and Build ArduPilot SITL

```bash
# Clone ArduPilot
git clone https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
git submodule update --init --recursive

# Install dependencies
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# Build ArduCopter (takes a few minutes)
./waf configure --board sitl
./waf copter
```

### 4 — Install Gazebo and the ArduPilot Gazebo Plugin

```bash
# Add Gazebo APT source
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
  -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update
sudo apt install gz-harmonic -y

# Install plugin build dependencies
sudo apt install libgz-sim8-dev rapidjson-dev -y
sudo apt install libopencv-dev libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad \
  gstreamer1.0-libav gstreamer1.0-gl -y

# Clone and build the plugin
git clone https://github.com/ArduPilot/ardupilot_gazebo.git ~/ardupilot_gazebo
cd ~/ardupilot_gazebo
mkdir build && cd build
export GZ_VERSION=harmonic
cmake ..
make -j$(nproc)
sudo make install
```

### 5 — Clone This Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Cybverse-Pkians/c-_obstacle_avoidance_drone_depthcam.git obstacle_avoidance_drone
```

### 6 — Build the ROS 2 Package

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Running the Simulation

You will need **three separate terminal windows**. Open each one and source your workspace first:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

#### Terminal 1 — Launch Gazebo

```bash
gz sim ~/ros2_ws/src/obstacle_avoidance_drone/obstacle_avoidance/worlds/obstacle_world.sdf
```

#### Terminal 2 — Start ArduPilot SITL

```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

Wait until you see `APM: EKF2 IMU0 is using GPS` in the console before proceeding.

#### Terminal 3 — Launch the Obstacle Avoidance Node

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch obstacle_avoidance obstacle_avoidance.launch.py
```

The drone will arm, take off, and begin navigating. When the depth camera detects an obstacle within the configured threshold distance, the avoidance manoeuvre executes automatically.

---

## ⚙️ Configuration

Key parameters can be tuned in `obstacle_avoidance/config/params.yaml`:

| Parameter | Default | Description |
|---|---|---|
| `obstacle_threshold_m` | `2.0` | Distance (metres) at which avoidance triggers |
| `avoidance_speed` | `0.5` | Lateral avoidance velocity (m/s) |
| `cruise_altitude_m` | `5.0` | Target flight altitude |
| `depth_topic` | `/camera/depth/image_raw` | ROS 2 depth image topic |
| `fcu_url` | `udp://127.0.0.1:14550` | MAVLink endpoint for MAVROS |

After editing, rebuild and re-source:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 📁 Repository Structure

```
obstacle_avoidance/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── params.yaml          # Tunable parameters
├── include/
│   └── obstacle_avoidance/  # C++ header files
├── launch/
│   └── obstacle_avoidance.launch.py
├── src/
│   └── obstacle_avoidance_node.cpp   # Core avoidance logic
└── worlds/
    └── obstacle_world.sdf    # Gazebo world with obstacles
```

---

## 🛠️ Troubleshooting

**Drone does not arm**
Make sure SITL has finished initialising (EKF healthy message in the console) before launching the ROS 2 node. Check that MAVROS is connected: `ros2 topic echo /mavros/state`.

**Depth image topic not publishing**
Verify the Gazebo depth camera plugin loaded correctly. Run `gz topic -l` and confirm the camera topic is present, then check that the topic name matches `depth_topic` in `params.yaml`.

**Build fails with missing headers**
Run `rosdep install --from-paths src --ignore-src -r -y` again inside `~/ros2_ws` and ensure `libgz-sim8-dev` is installed.

**MAVROS GeographicLib error**
Re-run: `sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh`

---

## 🤝 Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/my-feature`)
3. Commit your changes (`git commit -m 'Add my feature'`)
4. Push to the branch (`git push origin feature/my-feature`)
5. Open a Pull Request

---

## 📄 License

This project is licensed under the **GNU General Public License v3.0** — see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgements

- [ArduPilot](https://ardupilot.org/) — open-source autopilot firmware and SITL
- [Open Robotics](https://openrobotics.org/) — ROS 2 and Gazebo
- [MAVROS](https://github.com/mavlink/mavros) — MAVLink to ROS 2 bridge

---

*Built by [Cybverse-Pkians](https://github.com/Cybverse-Pkians)*
