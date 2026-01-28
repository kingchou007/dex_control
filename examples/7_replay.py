import time
import csv
from termcolor import cprint
from dex_control.robot.robot_client import FrankaRobotClient

# TODO: improve this
CSV_PATH = "examples/states.csv"  # Change this to your CSV path
RATE_HZ = 5.0  # Frequency of pose updates (matches your data collection)

def read_poses_from_csv(csv_path):
    poses = []
    with open(csv_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile, delimiter=',')
        for row in reader:
            # Extract position and quaternion from CSV columns
            translation = [
                float(row['PX']),
                float(row['PY']),
                float(row['PZ'])
            ]
            quaternion = [
                float(row['QX']),
                float(row['QY']),
                float(row['QZ']),
                float(row['QW'])
            ]
            poses.append((translation, quaternion))
    return poses


def main():
    cprint("=== Starting Franka Robot Trajectory Replay ===\n", "cyan", attrs=["bold"])

    robot = FrankaRobotClient(server_addr="tcp://192.168.1.6:4242")

    poses = read_poses_from_csv(CSV_PATH)
    cprint(f"Loaded {len(poses)} poses from CSV.", "green")

    delay = 1.0 / RATE_HZ  # Delay between poses

    for i, (translation, quaternion) in enumerate(poses):
        cprint(f"Moving to pose {i+1}/{len(poses)}:", "yellow")
        cprint(f" Translation: {translation}", "cyan")
        cprint(f" Quaternion: {quaternion}\n", "cyan")

        # Move end-effector to the desired pose (absolute)
        robot.move_ee_pose(translation + quaternion, asynchronous=True, delta=False)

        # Sleep to maintain the replay rate
        time.sleep(delay)

    cprint("=== Trajectory replay complete ===", "cyan", attrs=["bold"])

if __name__ == "__main__":
    main()
