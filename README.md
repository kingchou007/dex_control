# dex-control

Franka Research 3 Control Framework supporting teleoperation, real-time control.

**Note:** This project is actively developed and currently intended for internal use within the Duke Dexterity Lab. This is the franky version (it's very easy to install, so we do not provide Docker), which you can use for quick testing and simple control. If you need advanced APIs, please use the [ROS2](https://github.com/Robot-Dexterity-Lab/dex-control/tree/ros2) branch (under development).

## TODO

- [x] Communication
- [x] Basic control API
- [x] Integrate teleoperation and data collection workflows  
    - [x] Add error handling, support for reconnecting, recalling, and rerunning processes
- [x] Add support for inference pipeline
- [x] Perform comprehensive real robot testing
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

### Eye to Hand Calibration

This guide explains how to run the eye-to-hand calibration between a camera and robot arm using `example/8_cali_eye2hand.py`.

#### Prerequisites
Make sure you have:

- A RealSense camera mounted in a fixed position observing the robot workspace
- A Franka robot arm can read tcp pose
- An [ArUco marker](https://chev.me/arucogen/) (Dic: Original, ID: 582, Size: 100mm) attached to the robot end-effector
- ROS2 environment set up

#### Callibration Steps
1. Open a new terminal and launch the RealSense camera node:
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

2. Open a new terminal and launch the rqt_image_view:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_viewew
```

3. Open a new terminal to connect to the Franka NUC and execute the robot control wrapper:
```bash
ssh labuser@192.168.1.6
conda activate dex-control
cd dex-control
python dex_control/robot/franka_wrapper.py 
```

4. Open a new terminal to start '4_kinesthetic_teaching.py':
```bash
conda activate dex-control
cd dex-control/examples
python 4_kinesthetic_teaching.py 
```

5. Open a new terminal to start eye-to-hand calibration:

If this is your first time running the calibration script, initialize the virtual environment and install the required dependencies:
```bash
python3 -m venv ~/calib_venv --system-site-packages
source ~/calib_venv/bin/activate
pip install opencv-contrib-python scipy click numpy
```

Proceed to execute the calibration script:
```bash
source ~/calib_venv/bin/activate 
cd dex-control/examples
python 8_cali_eye2hand.py 
```

> **Important:** Prior to execution, ensure that `self.marker_size` within the script is configured to match the exact physical edge length of your ArUco marker (in meters).
> 
> **Note:** Press `ENTER` in the terminal to capture discrete calibration views, and `q` to terminate data collection and compute the final transformation.

The procedure generates the following structured dataset containing the calibration artifacts and raw data:

```text
calibration_results/
└── <timestamp>/
    ├── rgb/                    <-- Raw RGB captures per frame
    │   ├── 0.jpg
    │   ├── 1.jpg
    │   └── ...
    ├── vis/                    <-- Annotated frames with projected coordinate axes
    │   ├── vis_view_3.jpg
    │   ├── vis_view_4.jpg
    │   └── ...
    ├── per_frame_results/      <-- Incremental transformation matrices
    │   ├── 3views_c2r.npy
    │   ├── 3views_c2r.csv
    │   └── ...
    ├── trajectory.csv          <-- End-effector poses (translation and quaternion)
    ├── c2r.csv                 <-- Final Camera-to-Robot base transformation matrix
    ├── c2r.npy                 
    ├── o2cs.npy                <-- Raw Object-to-Camera matrices
    └── g2rs.npy                <-- Raw Gripper-to-Robot base matrices
```

---
**Author:** [Jinzhou Li](https://kingchou007.github.io/)

