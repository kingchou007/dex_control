"""Kinesthetic Teaching Test (soft impedance)

Same as 4_kinesthetic_teaching.py but with lower default impedance for testing.

Usage:
    python examples/5_test.py
    python examples/5_test.py teaching.impedance=5.0
"""

import hydra
from omegaconf import DictConfig

from dex_control.teleop.kinesthetic_teaching import KinestheticTeaching


@hydra.main(config_path="../config", config_name="kinesthetic_teaching", version_base=None)
def main(cfg: DictConfig):
    """Kinesthetic Teaching Test - soft impedance mode."""
    teaching = KinestheticTeaching(
        ip=cfg.server.host,
        port=cfg.server.port,
        duration=cfg.teaching.duration,
        impedance=1.0,
        zmq_port=cfg.teaching.zmq_port,
    )
    teaching.run()


if __name__ == "__main__":
    main()
