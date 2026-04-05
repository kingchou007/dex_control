# Robot PC (NUC) Setup

This guide covers how to set up the Franka ROS 2 controllers on the Robot PC (NUC).

## Prerequisites

- Docker installed on the NUC
- Network access to the Franka robot

## Installation

Install the ROS 2 controller package:

```bash
ssh <ROBOT_PC_IP>
git clone https://github.com/kingchou007/fr3_ros2_controllers.git
cd fr3_ros2_controllers
```

## Docker Setup

Build and launch the container:

```bash
docker compose build
docker compose up launch_franka
```

This starts the Franka ROS 2 controller stack inside Docker. Once running, you can start the `franka_wrapper.py` server on the NUC as described in the main [README](../README.md#quick-start).
