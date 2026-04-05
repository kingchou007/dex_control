# dex-control
**A Python Franka Control Framework**

Purely Python Franka Research 3 control framework for teleoperation, data collection, and real-time control. If you need more advanced control with ROS 2 integration, see our ROS 2 control implementation.

## Features

- **ROS-free** — pure Python, no ROS dependency;
- **Client-server architecture** — server on NUC or local machine, control from any networked client;
- **Teleoperation** — SpaceMouse, motion tracker, kinesthetic teaching, and VR;
- **Data collection** — multi-camera (ZED, GoPro, RealSense) recording with synchronized trajectories
- **Visualization** — browser-based visualizer
- **Gripper support** — Robotiq/Franka gripper control
- **Calibration** — hand-eye calibration utilities

## Quick Start

### Install

```bash
git clone --recurse-submodules https://github.com/Robot-Dexterity-Lab/dex-control
cd dex-control
pip install .
```

### Run

```bash
# 1. Sync code to NUC (after any local changes)
bash sync_infra.sh

# 2. Start server on NUC
python scripts/start_robot_server.py

# 3. Connect from client
python scripts/start_client.py
```

### Examples

| Script | Description |
|--------|-------------|
| `examples/0_basic_control.py` | Basic end-effector control |
| `examples/1_spacemouse_teleop.py` | SpaceMouse teleoperation |
| `examples/2_tracker_teleop.py` | Hand tracker teleoperation |
| `examples/3_zmq_bridge.py` | ZMQ bridge for external control |
| `examples/4_kinesthetic_teaching.py` | Kinesthetic teaching |
| `examples/6_reset.py` | Reset robot to home pose |
| `examples/7_replay.py` | Replay a recorded trajectory |

## Documentation

- [Robot PC Setup](docs/ROBOT_PC.md) — NUC-side server and controller setup
- [Data Collection](docs/DATA_COLLECT.md) — recording SOP, camera setup
- [Camera Calibration](scripts/calibration/CAMERA_CALI.md) — eye-to-hand calibration
- [Configuration](config/) — YAML configs for robot and kinesthetic teaching

## Project Structure

```
dex-control/
├── config/             # YAML configs
├── dex_control/
│   ├── robot/          # Server, client, robot interface
│   ├── teleop/         # Teleoperation backends
│   ├── camera/         # Camera drivers (RealSense, ZED, GoPro)
│   ├── gripper/        # Gripper control (Robotiq)
│   ├── data/           # Data processing
│   └── utils/          # Shared utilities, visualization
├── assets/             # Robot URDF and meshes
├── examples/           # Usage examples
├── scripts/            # Setup, calibration, data collection
└── docs/               # Documentation
```

## Acknowledgments

Built by [Jinzhou Li](https://kingchou007.github.io/) @ [Duke Dexterity Lab](https://github.com/Robot-Dexterity-Lab). Built on [franky](https://github.com/TimSchneider42/franky) for low-level control, communication inspired by [EVA](https://github.com/willjhliang/eva)'s ZeroRPC setup, and data pipeline inspired by [DROID](https://github.com/droid-dataset/droid). If you use our codebase, please consider citing them. You can find the cite button in the sidebar.