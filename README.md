# dex-control

Franka Research 3 Control Framework supporting teleoperation, data collection, and real-time control.

## Features

- **Client-server architecture** — run the server on the NUC (robot PC) and control from any networked client
- **Multiple teleoperation modes** — SpaceMouse, hand tracker (FACTr), kinesthetic teaching
- **Data collection pipeline** — multi-camera recording (RealSense, ZED, GoPro) with synchronized trajectories
- **Gripper support** — Robotiq gripper control via serial
- **Camera calibration** — eye-to-hand calibration utilities

## Installation

### Client (Laptop / Desktop)

```bash
git clone --recurse-submodules https://github.com/Robot-Dexterity-Lab/dex-control
cd dex-control
conda create -n dex-control python=3.10
conda activate dex-control
pip install .
```

### Robot PC (NUC)

```bash
bash sync_infra.sh   # edit LOCAL_DIR and NUC_DIR inside sync_infra.sh as needed
cd dex-control
conda create -n dex-control python=3.10
conda activate dex-control
pip install .
bash scripts/install.sh
```

> **Tip:** Use `--recurse-submodules` during cloning to automatically initialize submodules. Otherwise, run `git submodule update --init --recursive` after cloning.

> **Note:** Run `sync_infra.sh` to synchronize code between your local machine and the NUC before making edits.

For the NUC-side ROS 2 controller setup, see [docs/ROBOT_PC.md](docs/ROBOT_PC.md).

## Usage

We recommend the following tmux layout:
```
+-------------------------+-------------------------+
|                         |                         |
|      Server (NUC)       |     Runner (Laptop)     |
+-------------------------+                         |
|    Scripts (Laptop)     |                         |
|                         |                         |
+-------------------------+-------------------------+
```

### 1. Start the Server (on NUC)

```bash
conda activate dex-control
python dex_control/robot/franka_wrapper.py --ip <ROBOT_IP> --controller-type joint_impedance
```

### 2. Connect from Client (on Laptop)

```python
from dex_control.robot.robot_client import FrankaRobotClient

robot = FrankaRobotClient(server_addr="tcp://<NUC_IP>:4242")

# Get robot state
state = robot.get_ee_pose()
print("Joints:", robot.get_joint_positions())

# Relative move: shift 10 cm in x
robot.move_ee_pose([0.1, 0.0, 0.0], delta=True)

# Absolute move: [x, y, z, qx, qy, qz, qw]
robot.move_ee_pose([0.55, 0.0, 0.4, 0.0, 0.0, 1.0, 0.0], delta=False)
```

### Examples

| Script | Description |
|--------|-------------|
| `examples/0_basic_control.py` | Basic end-effector control |
| `examples/1_spacemouse_teleop.py` | SpaceMouse teleoperation |
| `examples/2_tracker_teleop.py` | Hand tracker teleoperation |
| `examples/3_zmq_bridge.py` | ZMQ bridge for external communication |
| `examples/4_kinesthetic_teaching.py` | Kinesthetic teaching / demonstration recording |
| `examples/6_reset.py` | Reset robot to home pose |
| `examples/7_replay.py` | Replay a recorded trajectory |

```bash
# Basic control
python examples/0_basic_control.py

# SpaceMouse teleoperation
python examples/1_spacemouse_teleop.py --ip <ROBOT_IP>
```

Replace `<ROBOT_IP>` with your NUC or robot server's IP address.

### Data Collection

See [docs/DATA_COLLECT.md](docs/DATA_COLLECT.md) for the full data collection SOP, including GoPro, RealSense, and recording setup.

```bash
python scripts/data_collect.py
```

### Camera Calibration

```bash
python scripts/calibration/kinesthetic_teaching_cali.py
python scripts/calibration/cali_eye2hand.py
```

See [scripts/calibration/CAMERA_CALI.md](scripts/calibration/CAMERA_CALI.md) for detailed instructions.

### CLI Client

```bash
python dex_control/robot/robot_client.py --ip <NUC_IP>
```

### Reset Robot

```bash
python scripts/reset_robot.py
```

## Project Structure

```
dex-control/
├── dex_control/
│   ├── robot/          # Server, client, and robot interface
│   ├── teleop/         # Teleoperation backends (SpaceMouse, FACTr)
│   ├── camera/         # Camera drivers (RealSense, ZED, GoPro)
│   ├── gripper/        # Gripper control (Robotiq)
│   ├── data/           # Data processing utilities
│   └── utils/          # Shared utilities
├── examples/           # Usage examples
├── scripts/            # Setup, calibration, and data collection scripts
├── docs/               # Additional documentation
└── assets/             # Media assets
```

## Acknowledgments

This repo is built with [franky](https://github.com/TimSchneider42/franky) as the low-level Franka control interface, and draws inspiration from [DROID](https://github.com/droid-dataset/droid) and [EVA](https://github.com/willjhliang/eva) to simplify the codebase.
