from dex_control.robot.client import NucRobotClient

# Default joint positions for reset (modify as needed)
DEFAULT_JOINTS = [
    0.09162,  # Joint 1
    -0.19826,  # Joint 2
    -0.01990,  # Joint 3
    -2.47323,  # Joint 4
    -0.01307,  # Joint 5
    2.30397,  # Joint 6
    0.84809,  # Joint 7
]


def reset_robot(joints=DEFAULT_JOINTS, duration=5.0):
    client = NucRobotClient()
    print("Resetting robot...")
    client.reset_to_joints(joints, duration)
    print("Reset done.")


if __name__ == "__main__":
    reset_robot()
