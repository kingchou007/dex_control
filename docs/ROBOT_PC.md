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

## Network Security Note

The ZeroRPC server binds to `0.0.0.0` with no authentication. This is intended for isolated lab networks with a direct connection between the workstation and the NUC. If your network is shared, restrict access with a firewall rule:

```bash
# Allow only your workstation IP
sudo ufw allow from <WORKSTATION_IP> to any port 4242
sudo ufw deny 4242
```
