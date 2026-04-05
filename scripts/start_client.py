"""Start Franka robot client.

Usage:
    # Auto-connect from config/robot.yaml
    python scripts/start_client.py

    # Override address
    python scripts/start_client.py --addr tcp://192.168.1.6:4242

    # Use custom config
    python scripts/start_client.py --config config/my_robot.yaml
"""

import subprocess
import click
import yaml
from pathlib import Path

from dex_control.robot.robot_client import FrankaRobotClient

SYNC_SCRIPT = Path(__file__).parent.parent / "sync_infra.sh"

DEFAULT_CONFIG = Path(__file__).parent.parent / "config" / "robot.yaml"


def load_server_addr(config_path: str = None) -> str:
    """Load server address from robot.yaml."""
    path = Path(config_path) if config_path else DEFAULT_CONFIG
    try:
        with open(path, 'r') as f:
            cfg = yaml.safe_load(f)
        ip = cfg["server"]["ip"]
        port = cfg["server"]["port"]
        remote = cfg["server"].get("remote", True)
        host = ip if remote else "localhost"
        return f"tcp://{host}:{port}"
    except (FileNotFoundError, KeyError):
        return "tcp://192.168.1.7:4242"


@click.command()
@click.option("--config", default=None, help="Path to robot.yaml config")
@click.option("--addr", default=None, help="Override server address (tcp://ip:port)")
@click.option("--no-sync", is_flag=True, help="Skip syncing to NUC")
def main(config, addr, no_sync):
    """Connect to robot server and print state."""
    if not no_sync and SYNC_SCRIPT.exists():
        print("[Sync] Syncing to NUC...")
        subprocess.run(["bash", str(SYNC_SCRIPT)], check=False)

    if addr is None:
        addr = load_server_addr(config)

    robot = FrankaRobotClient(server_addr=addr)
    try:
        print(f"Joints:  {robot.get_joint_positions()}")
        print(f"EE Pose: {robot.get_ee_pose()}")
        try:
            print(f"Gripper: {robot.get_gripper_width():.4f} m")
        except Exception:
            print("Gripper: not available")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        robot.close()


if __name__ == "__main__":
    main()
