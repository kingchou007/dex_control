"""Kinesthetic Teaching Launch Script

Thin launcher that loads config from YAML and runs kinesthetic teaching.

Usage:
    python examples/4_kinesthetic_teaching.py
    python examples/4_kinesthetic_teaching.py server.host=192.168.1.7
    python examples/4_kinesthetic_teaching.py teaching.impedance=10.0
"""

import hydra
from omegaconf import DictConfig

from dex_control.teleop.kinesthetic_teaching import KinestheticTeaching


@hydra.main(config_path="../config", config_name="kinesthetic_teaching", version_base=None)
def main(cfg: DictConfig):
    """Kinesthetic Teaching - Enable gravity compensation for manual guidance."""
    teaching = KinestheticTeaching(
        ip=cfg.server.host,
        port=cfg.server.port,
        duration=cfg.teaching.duration,
        impedance=cfg.teaching.impedance,
        zmq_port=cfg.teaching.zmq_port,
        publish_rate=cfg.teaching.publish_rate,
    )
    teaching.run()


if __name__ == "__main__":
    main()
