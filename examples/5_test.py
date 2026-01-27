"""Kinesthetic Teaching Example for Franka Robot

Enables gravity compensation mode allowing manual guidance of the robot arm.
"""

import time
import click
import numpy as np
from termcolor import cprint

from dex_control.robot.robot_client import FrankaRobotClient


DEFAULT_IP = "192.168.1.6"
DEFAULT_PORT = 4242


@click.command()
@click.option("--ip", default=DEFAULT_IP, help="Robot server IP address")
@click.option("--port", default=DEFAULT_PORT, help="Robot server port")
@click.option("--stiffness", default=200.0, help="Cartesian stiffness (lower=more compliant)")
@click.option("--duration", default=300.0, help="Duration in seconds")
def main(ip, port, stiffness, duration):
    """Kinesthetic Teaching with Cartesian Impedance."""

    server_addr = f"tcp://{ip}:{port}"

    cprint("=== Kinesthetic Teaching Mode ===\n", "cyan", attrs=["bold"])

    try:
        cprint(f"Connecting to robot at {server_addr}...", "yellow")
        robot = FrankaRobotClient(server_addr=server_addr)
        cprint("Connected!\n", "green")
    except Exception as e:
        cprint(f"Error connecting to robot: {e}", "red")
        return

    # 设置 Cartesian Impedance（低刚度 = 容易推动）
    cart_stiffness = [stiffness, stiffness, stiffness, 20.0, 20.0, 20.0]
    cprint(f"Setting cartesian impedance to {cart_stiffness}", "yellow")
    robot.set_cartesian_impedance(cart_stiffness)

    cprint(f"\nStarting kinesthetic teaching for {duration} seconds...", "yellow")
    cprint("You can now manually guide the robot arm.\n", "green", attrs=["bold"])
    cprint("Press Ctrl+C to stop and exit.\n", "magenta")

    try:
        start_time = time.time()
        while time.time() - start_time < duration:
            joints = robot.get_joint_positions()
            ee_pose = robot.get_ee_pose()

            joint_str = ", ".join([f"{j:.3f}" for j in joints])
            pos_str = f"[{ee_pose['t'][0]:.3f}, {ee_pose['t'][1]:.3f}, {ee_pose['t'][2]:.3f}]"

            elapsed = time.time() - start_time
            remaining = duration - elapsed

            print(f"\r[{remaining:.0f}s] Joints: [{joint_str}] | EE: {pos_str}", end="", flush=True)

            time.sleep(0.1)

    except KeyboardInterrupt:
        cprint("\n\nStopping...", "yellow")
    finally:
        robot.stop_motion()
        cprint("Kinesthetic teaching ended.", "green")
        robot.close()


if __name__ == "__main__":
    main()