"""Kinesthetic Teaching Test (soft impedance)

Same as 4_kinesthetic_teaching.py but with lower default impedance for testing.

Usage:
    python examples/5_test.py
    python examples/5_test.py --ip 192.168.1.7 --impedance 5.0
"""

import click
import yaml

from dex_control.teleop.kinesthetic_teaching import KinestheticTeaching


DEFAULT_CONFIG = "config/kinesthetic_teaching.yaml"


def load_config(config_path: str) -> dict:
    try:
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        return {}


@click.command()
@click.option("--config", "config_path", default=DEFAULT_CONFIG, help="Path to YAML config file")
@click.option("--ip", default=None, help="Robot server IP address")
@click.option("--port", default=None, type=int, help="Robot server port")
@click.option("--duration", default=None, type=float, help="Duration in seconds")
@click.option("--impedance", default=1.0, type=float, help="Joint impedance (default: 1.0 soft)")
@click.option("--zmq-port", default=5557, type=int, help="ZMQ PUB port")
def main(config_path, ip, port, duration, impedance, zmq_port):
    """Kinesthetic Teaching Test - soft impedance mode."""

    cfg = load_config(config_path)
    server_cfg = cfg.get("server", {})
    teaching_cfg = cfg.get("teaching", {})

    teaching = KinestheticTeaching(
        ip=ip or server_cfg.get("ip", "192.168.1.6"),
        port=port or server_cfg.get("port", 4242),
        duration=duration or teaching_cfg.get("duration", 300.0),
        impedance=impedance,
        zmq_port=zmq_port,
    )
    teaching.run()


if __name__ == "__main__":
    main()
