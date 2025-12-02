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

Reset script: `python scripts/reset_robot.py`

## License

MIT License - see [LICENSE](LICENSE) file for details.

## Acknowledgments
This project uses the official [libfranka](https://github.com/frankarobotics/libfranka) library from Franka Robotics, with [franky](https://github.com/TimSchneider42/franky) as the Python interface. It also builds on data frameworks such as [droid](https://github.com/droid-dataset/droid) and [eva](https://github.com/willjhliang/eva) from Upen, along with other internal and open-source libraries for perception, planning, and teleoperation.
