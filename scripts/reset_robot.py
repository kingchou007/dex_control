from dex_control.robot.robot_client import FrankaRobotClient

# Default joint positions for reset (modify as needed)
DEFAULT_JOINTS = [
    0.09162,   # Joint 1
    -0.19826,  # Joint 2
    -0.01990,  # Joint 3
    -2.47323,  # Joint 4
    -0.01307,  # Joint 5
    2.30397,   # Joint 6
    0.84809,   # Joint 7
]

def reset_robot(joints=DEFAULT_JOINTS):
    robot = FrankaRobotClient(server_addr="tcp://192.168.1.7:4242", franka_hand=True)
    print("Resetting robot...")
    robot.move_joints(joints, asynchronous=False)
    if robot.franka_hand:
        robot.release_object()
    print("Reset done.")

if __name__ == "__main__":
    reset_robot()
