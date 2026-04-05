"""Start Franka robot RPC server with Hydra config.

Usage:
    # Default config (config/robot.yaml)
    python scripts/start_robot_server.py

    # Override params from CLI
    python scripts/start_robot_server.py server.ip=192.168.1.50 server.port=5000

    # Use a different config file (config/my_robot.yaml)
    python scripts/start_robot_server.py --config-name=my_robot
"""

import hydra
import zerorpc
from omegaconf import DictConfig

from dex_control.robot.franka_wrapper import FrankaWrapper


@hydra.main(config_path="../config", config_name="robot", version_base=None)
def main(cfg: DictConfig):
    wrapper = FrankaWrapper(cfg)

    port = cfg.server.port
    s = zerorpc.Server(wrapper, heartbeat=None)
    s.bind(f"tcp://0.0.0.0:{port}")
    wrapper.log.info(f"Franka Server listening on tcp://0.0.0.0:{port}")
    wrapper.log.info(f"Teleoperation: {cfg.teleop}")
    s.run()


if __name__ == "__main__":
    main()
