# Teleoperation

This guide covers the supported teleoperation methods in dex-control.

## Overview

```
Device Input → Signal Processing → FrankaRobotClient → Robot Server → Franka Robot
                                                            ↓
                                                      ZMQ State Pub (port 5557)
```

All teleop methods share the same client-server architecture: the input device runs on your workstation, sends commands via `FrankaRobotClient` (ZeroRPC), and the server on the NUC executes them on the robot.

## Prerequisites

1. Robot server running on NUC (see [Robot PC Setup](ROBOT_PC.md))
2. `pip install .` (core install)

## Methods

### 1. SpaceMouse

6-DOF USB input device for end-effector velocity control.

```bash
python examples/1_spacemouse_teleop.py --ip <ROBOT_IP> --translation-scale 1.0
```

- Reads 6-DOF (translation + rotation) via USB HID
- Left/right buttons for binary gripper open/close
- Publishes robot state on ZMQ port 5557

**Requirements:** `hidapi` (included in core install), SpaceMouse Wireless (VID: 9583, PID: 50741)

### 2. Motion Tracker

Pose tracking via ZMQ (e.g., Vive tracker).

```bash
python examples/2_tracker_teleop.py --ip <ROBOT_IP> --tracker-host localhost --tracker-port 5558
```

- Receives tracker pose via ZMQ SUB on port 5558
- Expected message format: `{"position": [x, y, z], "orientation": [qx, qy, qz, qw]}`
- Coordinate frame transformation and delta clamping for safety
- Relative pose from initial calibration

**Requirements:** An external tracker publisher sending pose data to the ZMQ port.

### 3. Kinesthetic Teaching

Manual arm guidance with gravity compensation.

```bash
python examples/4_kinesthetic_teaching.py --config config/kinesthetic_teaching.yaml --duration 300
```

- Robot enters gravity compensation mode (compliant joints)
- Configurable joint impedance (0 = soft, 50 = stiff)
- Publishes joint/EE state via ZMQ at configured rate

**Config:** [`config/kinesthetic_teaching.yaml`](../config/kinesthetic_teaching.yaml)

```yaml
server:
  ip: "192.168.1.6"
  port: 4242
teaching:
  duration: 300.0
  impedance: 5.0
  publish_rate: 100.0
```

### 4. Quest 3 VR Controllers

Stream Quest 3 controller data (pose + buttons) via UDP.

```bash
# First, fetch the submodule
git submodule update --init dex_control/dependencies/quest3-controller-tracker

# Run the receiver
python dex_control/dependencies/quest3-controller-tracker/receiver.py --port 7000
```

- Unity app on Quest 3 sends JSON snapshots at 15 Hz over UDP
- Per-hand: position, rotation, velocity, buttons (trigger, grip, primary, secondary, thumbstick, menu)
- See [`quest3-controller-tracker/README.md`](../dex_control/dependencies/quest3-controller-tracker/README.md) for Unity setup

### 5. ZMQ Bridge (External Control)

REQ/REP socket for programmatic control from external processes.

```bash
python examples/3_zmq_bridge.py --ip <ROBOT_IP> --port 5556
```

- Commands: `set_pose`, `get_pose`, `set_joints`, `get_joints`
- Useful for integrating with external planners or RL policies

### 6. FACTR (Leader-Follower)

Force-Attending Curriculum Training with a leader arm via ROS 2.

- Requires ROS 2 setup
- Leader arm publishes joint commands on `/factr/joint_commands`
- Follower arm streams joint states on `/franka/joint_states`
- See `dex_control/teleop/factr/` for details

## Port Reference

| Port | Protocol | Usage |
|------|----------|-------|
| 4242 | ZeroRPC  | Robot server |
| 5556 | ZMQ REP  | ZMQ bridge action server |
| 5557 | ZMQ PUB  | Robot state publisher |
| 5558 | ZMQ SUB  | Tracker pose input |
| 7000 | UDP      | Quest 3 controller receiver |

## Robot Config

Teleop-relevant settings in [`config/robot.yaml`](../config/robot.yaml):

```yaml
teleop: true
controller:
  type: "joint_impedance"
  joint_impedance: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0]
  cartesian_impedance: [1000.0, 1000.0, 1000.0, 30.0, 30.0, 30.0]
```
