# dex-control

Franka Research 3 Control Framework supporting teleoperation, real-time control.

**Author:** [Jinzhou Li](https://kingchou007.github.io/)

**Note:** This project is actively developed and currently intended for internal use within the Duke Dexterity Lab. This is the franky version (it's very easy to install, so we do not provide Docker), which you can use for quick testing and simple control. If you need advanced APIs, please use the [ROS2](https://github.com/Robot-Dexterity-Lab/dex-control/tree/ros2) branch (under development).


![Demo](assets/example.gif)


## TODO

- [x] Communication
- [x] Basic control API
- [x] Integrate teleoperation and data collection workflows  
    - [x] Add error handling, support for reconnecting, recalling, and rerunning processes
- [x] Add support for inference pipeline
- [ ] Perform comprehensive real robot testing
- [ ] Write a detailed installation guide from scratch

## Installation

1. **Clone the repository:**

   On your **laptop or desktop** (local development machine):
   ```bash
   git clone --recurse-submodules https://github.com/Robot-Dexterity-Lab/dex-control
   cd dex-control
   conda create -n dex-control python=3.10
   conda activate dex-control
   pip install .
   ```

   On the **NUC** (robot/server PC):
   ```bash
   bash sync_infra.sh   # (edit LOCAL_DIR and NUC_DIR inside sync_infra.sh as needed)
   cd dex-control
   conda create -n dex-control python=3.10
   conda activate dex-control
   pip install .
   bash scripts/install.sh
   ```

   > **Tip:** Use `--recurse-submodules` during cloning to automatically initialize submodules. Otherwise, after cloning, run `git submodule update --init --recursive`.

   > **Note:** Before making any edits, make sure to run `sync_infra.sh` to synchronize code between your local machine and the NUC.


2. (Recommended) Install all dependencies and setup in one step using the provided setup script:
```bash
bash scripts/install_all.sh 
```


## Usage

We recommend the following tmux setup:
```
+-------------------------+-------------------------+
|                         |                         |
|      Server (NUC)       |     Runner (Laptop)     |
+-------------------------+                         |
|    Scripts (Laptop)     |                         |
|                         |                         |
+-------------------------+-------------------------+
```

### 1. Start Up (on NUC/Robot PC)
1. On the NUC, run
```bash
conda activate dex-control
python dex_control/robot/franka_wrapper.py --ip <ROBOT_IP> --controller-type joint_impedance
```
2. On the laptop, run
```bash
conda activate dex-control
python basic_control/.py
```

### 2. Connect from Client (on your PC)
You can find additional usage examples inside the `examples` directory, such as:

```python
from dex_control.robot.robot_client import FrankaRobotClient

# Initialize client with the NUC's IP address
robot = FrankaRobotClient(server_addr="tcp://<NUC_IP>:4242")

# Get robot state
state = robot.get_ee_pose()
print("Joints:", robot.get_joint_positions())

# Move robot
robot.move_ee_pose([0.1, 0.0, 0.0], delta=True)  # Move x -> 10 cm

# Move end-effector to a target pose (translation + quaternion as a list)
# Example: Move to absolute position [x, y, z, qx, qy, qz, qw]
robot.move_ee_pose([0.55, 0.0, 0.4, 0.0, 0.0, 1.0, 0.0], delta=False)

# Move end-effector by a relative translation (in meters)
# Move z -> 10 cm down (if delta=True, negative z direction moves up)
# In absolute cmd, +z will move up, -z will move down.
robot.move_ee_pose([0.0, 0.0, 0.1], delta=True) 

# Move end-effector by a relative translation + orientation change (both relative)
# [dx, dy, dz, dqx, dqy, dqz, dqw], delta=True
relative_translation = [0.0, 0.10, 0.0]
relative_quaternion = [0.0, 0.0, 0.0, 1.0]  # No rotation (relative), of course you can use abs control 
robot.move_ee_pose(relative_translation + relative_quaternion, delta=True)
```

### Examples
```bash
python examples/0_basic_control.py
```

or run teleoperation with the SpaceMouse:

```bash
python examples/1_spacemouse_teleop.py --ip <ROBOT_IP>
```

Replace `<ROBOT_IP>` with your NUC or robot server's IP address as needed.



### CLI Client Usage

You can also use the client script directly from the command line:

```bash
python dex_control/robot/robot_client.py --ip <NUC_IP>
```

Reset script: `python scripts/reset_robot.py`

## Limitation
We currently only support joint and Cartesian position control. Because we use the [Python interface](https://github.com/TimSchneider42/franky), the control rate is usually below 100 Hz. I am now working on a new version that moves all computation to the C++ side with more robust communication, while Python only provides the API layer (a lite-[Polymetis](https://github.com/facebookresearch/polymetis) style design). This new version will also support [hybrid force control](https://github.com/yifan-hou/force_control).


## Acknowledgments
This project uses the official [libfranka](https://github.com/frankarobotics/libfranka) library from Franka Robotics, with [franky](https://github.com/TimSchneider42/franky) as the Python interface. It also builds on data frameworks such as [droid](https://github.com/droid-dataset/droid) and [eva](https://github.com/willjhliang/eva) from Upen, along with other internal and open-source libraries for perception, planning, and teleoperation.
