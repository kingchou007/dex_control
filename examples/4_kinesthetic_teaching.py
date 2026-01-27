"""Kinesthetic Teaching Example for Franka Robot

Enables gravity compensation mode allowing manual guidance of the robot arm.
Uses JointVelocityMotion with zero velocity and adjustable joint impedance.

Usage:
    python examples/4_kinesthetic_teaching.py [OPTIONS]

Options:
    --ip: Robot server IP address (default: 192.168.1.6)
    --port: Robot server port (default: 4242)
    --impedance: Joint impedance value for all joints (default: 0.0 for pure gravity comp)
    --duration: Duration in seconds (default: 300)
"""

import time
import click
from termcolor import cprint

from dex_control.robot.robot_client import FrankaRobotClient


DEFAULT_IP = "192.168.1.6"
DEFAULT_PORT = 4242


@click.command()
@click.option("--ip", default=DEFAULT_IP, help="Robot server IP address")
@click.option("--port", default=DEFAULT_PORT, help="Robot server port")
@click.option("--impedance", default=0.0, help="Joint impedance (0=full compliance, 50=soft, 300=stiff)")
@click.option("--duration", default=300.0, help="Duration in seconds")
def main(ip, port, impedance, duration):
    """Kinesthetic Teaching - Enable gravity compensation for manual guidance."""

    server_addr = f"tcp://{ip}:{port}"

    cprint("=== Kinesthetic Teaching Mode ===\n", "cyan", attrs=["bold"])

    try:
        cprint(f"Connecting to robot at {server_addr}...", "yellow")
        robot = FrankaRobotClient(server_addr=server_addr)
        cprint("Connected!\n", "green")
    except Exception as e:
        cprint(f"Error connecting to robot: {e}", "red")
        return

    # Set joint impedance
    impedance_values = [impedance] * 7
    cprint(f"Setting joint impedance to {impedance_values}", "yellow")
    robot.set_joint_impedance(impedance_values)

    if impedance == 0.0:
        cprint("Mode: Pure gravity compensation (fully compliant)", "cyan")
    else:
        cprint(f"Mode: Partial compliance (impedance={impedance})", "cyan")

    cprint(f"\nStarting kinesthetic teaching for {duration} seconds...", "yellow")
    cprint("You can now manually guide the robot arm.\n", "green", attrs=["bold"])

    try:
        robot.start_gravity_compensation(duration=duration)

        cprint("Press Ctrl+C to stop and exit.\n", "magenta")

        # Monitor and print joint positions periodically
        start_time = time.time()
        while time.time() - start_time < duration:
            joints = robot.get_joint_positions()
            ee_pose = robot.get_ee_pose()

            # Clear line and print current state
            joint_str = ", ".join([f"{j:.3f}" for j in joints])
            pos_str = f"[{ee_pose['t'][0]:.3f}, {ee_pose['t'][1]:.3f}, {ee_pose['t'][2]:.3f}]"

            elapsed = time.time() - start_time
            remaining = duration - elapsed

            print(f"\r[{remaining:.0f}s] Joints: [{joint_str}] | EE: {pos_str}", end="", flush=True)

            time.sleep(0.1)

    except KeyboardInterrupt:
        cprint("\n\nStopping kinesthetic teaching...", "yellow")
    finally:
        robot.stop_motion()
        cprint("Kinesthetic teaching ended.", "green")
        robot.close()


if __name__ == "__main__":
    main()
