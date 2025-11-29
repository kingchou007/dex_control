# dex-control

Franka Research 3 Control Framework supporting teleoperation, real-time control.

**Author:** Jinzhou Li

**Note:** This project is actively developed and currently intended for internal use within the Duke Dexterity Lab.


## Installation

1. Clone the repository and initialize submodules:
```bash
git clone <repository-url>
cd dex-control
git submodule update --init --recursive

conda create -n dex-control python=3.10
conda activate dex-control
```

2. (Recommended) Install all dependencies and setup in one step using the provided setup script:
```bash
bash scripts/install_all.sh  # (script coming soon)
```


## Usage

```python
from dex_control.robot.client import NucRobotClient

client = NucRobotClient(nuc_addr="tcp://:4242")
state = client.get_state()
client.reset_to_joints([0.0, -0.5, 0.0, -2.0, 0.0, 2.0, 0.8], duration=5.0)
```

Reset script: `python scripts/reset_robot.py`

## License

MIT License - see [LICENSE](LICENSE) file for details.

## Acknowledgments
This project is built using the official [libfranka](https://github.com/frankarobotics/libfranka) library from Franka Robotics.
