# Camera-Robot Calibration Guide

This guide explains how to run the eye-to-hand calibration between a camera and robot arm using `cali_eye2hand.py`.

## Environment Setup
```bash
sudo apt-get install ros-jazzy-sensor-msgs ros-jazzy-cv-bridge ros-jazzy-realsense2-camera
python3 -m venv ~/calib_venv --system-site-packages
source ~/calib_venv/bin/activate
pip install opencv-contrib-python scipy click numpy
```

## Prerequisites
Make sure you have:

- A RealSense camera mounted in a fixed position observing the robot workspace.
- A Franka robot arm capable of broadcasting its TCP pose.
- An [ArUco marker](https://chev.me/arucogen/) (Dictionary: Original, ID: 582, Size: 100mm) attached to the robot end-effector.
- A ROS 2 environment set up.

## Calibration Steps
1. Open a new terminal and launch the RealSense camera node:
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

2. Open a new terminal and launch `rqt_image_view`:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view
```

3. Open a new terminal to connect to the Franka_1 NUC and execute the robot control wrapper:
```bash
ssh labuser@192.168.1.6
conda activate dex-control
cd dex-control
python dex_control/robot/franka_wrapper.py 
```

4. Open a new terminal to start `4_kinesthetic_teaching.py`:
```bash
conda activate dex-control
cd dex-control/examples
python 4_kinesthetic_teaching.py 
```

5. Open a new terminal to start eye-to-hand calibration:
```bash
source ~/calib_venv/bin/activate 
cd dex-control/scripts/calibration
python cali_eye2hand.py 
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