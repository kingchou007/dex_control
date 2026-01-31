import time
from termcolor import cprint
from dex_control.robot.robot_client import FrankaRobotClient


def main():
    """Example usage of FrankaRobotClient."""
    # Connect to the robot server
    robot = FrankaRobotClient(server_addr="tcp://192.168.1.6:4242")
    
    # Set up your home position
    joint_motion =[ 1.77008981,  1.18702485, -1.49880186, -1.66540347,  1.10065954,  2.28065967, 0.9746598 ]
    robot.move_joints(joint_motion)


if __name__ == "__main__":
    main()

