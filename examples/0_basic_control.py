"""Example script showing how to use FrankaRobotClient from other files.

Author: Jinzhou Li

This script demonstrates how to control the Franka robot using the Franka Robot Control Server.
Before running this script, make sure the robot server(NUC) is running:
    python dex_control/robot/franka_wrapper.py

Usage:
    python examples/robot_example.py
"""

import time
from termcolor import cprint
from dex_control.robot.robot_client import FrankaRobotClient


def main():
    """Example usage of FrankaRobotClient."""
    # Connect to the robot server
    robot = FrankaRobotClient(server_addr="tcp://192.168.1.7:4242")
    
    cprint("=== Robot Control Example ===\n", "cyan", attrs=["bold"])

    # 1. Move to home position
    cprint("1. Moving to home position...", "yellow")
    home_joints = [0.0, 0.0, 0.0, -2.2, 0.0, 2.2, 0.7]
    robot.move_joints(home_joints, asynchronous=False)
    cprint("   Movement completed!\n", "green")
    time.sleep(0.5)
    
    # 2. Get current end-effector pose
    cprint("2. Getting current end-effector pose...", "yellow")
    pose = robot.get_ee_pose()
    cprint(f"   Translation: {pose['t']}", "cyan")
    cprint(f"   Quaternion: {pose['q']}\n", "cyan")
    
    # 3. Get joint positions
    cprint("3. Getting joint positions...", "yellow")
    joints = robot.get_joint_positions()
    cprint(f"   Joint angles: {joints}\n", "cyan")
    
    # 4. Move end-effector down (relative motion)
    cprint("4. Moving end-effector DOWN by 10cm...", "yellow")
    cprint("   Note: Positive z = DOWN (toward table)", "magenta")
    robot.move_ee_pose([0.0, 0.0, 0.1], asynchronous=False, delta=True)
    cprint("   Movement completed!\n", "green")
    time.sleep(0.5)
    
    # 5. Move end-effector up (relative motion)
    cprint("5. Moving end-effector UP by 10cm...", "yellow")
    cprint("   Note: Negative z = UP (away from table)", "magenta")
    robot.move_ee_pose([0.0, 0.0, -0.1], asynchronous=False, delta=True)
    cprint("   Movement completed!\n", "green")
    time.sleep(0.5)
    
    # 6. Fine movement example
    cprint("6. Fine movement: Moving UP by 1cm...", "yellow")
    robot.move_ee_pose([0.0, 0.0, -0.01], asynchronous=False, delta=True)
    cprint("   Movement completed!\n", "green")
    
    cprint("=== Example completed ===", "cyan", attrs=["bold"])


if __name__ == "__main__":
    main()

