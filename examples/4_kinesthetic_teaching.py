"""Kinesthetic Teaching Launch Script

Thin launcher that loads config from YAML and runs kinesthetic teaching.

Usage:
    python examples/4_kinesthetic_teaching.py
    python examples/4_kinesthetic_teaching.py --config config/kinesthetic_teaching.yaml
    python examples/4_kinesthetic_teaching.py --ip 192.168.1.7 --impedance 10.0
"""

import click
import yaml

from dex_control.teleop.kinesthetic_teaching import KinestheticTeaching


DEFAULT_CONFIG = "config/kinesthetic_teaching.yaml"


def load_config(config_path: str) -> dict:
    """Load configuration from YAML file."""
    try:
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        return {}


@click.command()
@click.option("--config", "config_path", default=DEFAULT_CONFIG, help="Path to YAML config file")
@click.option("--ip", default=None, help="Robot server IP address (overrides config)")
@click.option("--port", default=None, type=int, help="Robot server port (overrides config)")
@click.option("--duration", default=None, type=float, help="Duration in seconds (overrides config)")
@click.option("--impedance", default=None, type=float, help="Joint impedance (overrides config)")
@click.option("--zmq-port", default=5557, type=int, help="ZMQ PUB port for state streaming")
@click.option("--publish-rate", default=None, type=float, help="Publish rate in Hz (overrides config)")
def main(config_path, ip, port, duration, impedance, zmq_port, publish_rate):
    """Kinesthetic Teaching - Enable gravity compensation for manual guidance."""

    # Load YAML config
    cfg = load_config(config_path)
    server_cfg = cfg.get("server", {})
    teaching_cfg = cfg.get("teaching", {})

    # CLI args override YAML config
    teaching = KinestheticTeaching(
        ip=ip or server_cfg.get("host", "192.168.1.6"),
        port=port or server_cfg.get("port", 4242),
        duration=duration or teaching_cfg.get("duration", 300.0),
        impedance=impedance or teaching_cfg.get("impedance", 5.0),
        zmq_port=zmq_port,
        publish_rate=publish_rate or teaching_cfg.get("publish_rate", 100.0),
    )
    teaching.run()


if __name__ == "__main__":
    main()
