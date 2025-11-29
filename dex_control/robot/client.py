import zerorpc


class NucRobotClient:
    def __init__(self, nuc_addr: str = "tcp://192.168.1.7:4242"):
        """Connect to FrankaServer on NUC."""
        self.nuc_addr = nuc_addr
        self.client = zerorpc.Client()
        self.client.connect(self.nuc_addr)
        print(f"Connected to NUC robot server at {self.nuc_addr}")

    def get_state(self):
        """Get current robot state."""
        return self.client.get_state()

    def reset_to_joints(
        self, target_joint_positions: list[float], duration: float = 5.0
    ):
        """Move robot to target joint positions."""
        return self.client.reset_to_joints(target_joint_positions, duration)

    def move_joints(self, joint_positions: list[float]):
        pass

    def move_ee_pose(
        self, end_effector_position: list[float], end_effector_orientation: list[float]
    ):
        pass
