# Data Collection SOP

## TODO

- [] test, and add more option
- [] remove ros requirements


After initializing the tracker in rviz, reset the tracker pose:
```sh
cd ~/toolchanger_ws
$ source install/setup.bash
$ ros2 service call /reset_origin std_srvs/srv/Empty
```

```sh
cd ~/dex-control
$ python examples/7_reset.py 
$ python examples/2_tracker_teleop.py
```

## Collect Data:

### GoPro Setup

1. Install GoPro Labs [firmware](https://gopro.com/en/us/info/gopro-labs)
2. Scan the following QR code for clean HDMI output
../asset/QR-MHDMI1mV0r27Tp60fWe0hS0sLcFg1dV.png
3. Connect Elgato capture card to USB 3.0 port.
4. Run the following command:
(should exit any virtual environment)
```sh
conda deactivate
source install/setup.bash
cd ~/Documents/maniwav/scripts_real
python3 gopro_mp4.py
```
If gopro streaming is successful, a gui will show real-time streaming video.
If there is any issue, first try to unplug the Elgate cable and reconnect.

### Realsense D435 launch

```sh
conda deactivate
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

### Recording
```sh
conda deactivate
source install/setup.bash
$ python3 data_collect.py 
press Enter twice to start collecting data, and press Enter to stop Data Collection
```
