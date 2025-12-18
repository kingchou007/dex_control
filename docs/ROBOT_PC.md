## Installation (Robot PC)

Install the following packages:
- [fr3_ros2_controllers](https://github.com/kingchou007/fr3_ros2_controllers)

```bash
ssh robot_pc_ip
git clone https://github.com/kingchou007/fr3_ros2_controllers.git
cd fr3_ros2_controllers
``` 

### Docker Setup
Ensure Docker is installed, then build and launch the container:

```bash
docker compose build
docker compose up launch_franka # start
```