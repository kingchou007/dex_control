
import time
import numpy as np
from dex_control.robot.client import NucRobotClient  # 按你自己保存文件的名字来 import
from scipy.spatial.transform import Rotation as R

class SimpleRunner:
    def __init__(self, nuc_addr: str = "tcp://192.168.1.7:4242", hz: float = 5.0):
        """nuc_addr 是 NUC 的 IP 和 zerorpc 端口，hz 是轮询频率。"""
        self.client = NucRobotClient(nuc_addr)
        self.dt = 1.0 / hz

    def step_once(self):
        """拉一次状态，并做你想做的处理。"""
        state = self.client.get_state()
        # print(state["joint_positions"])
        # print(state.joint_velocities)
        # print(state.joint_torques)
        # print(state.external_joint_torques)
        print([round(v, 8) for v in state["end_effector_position"]])
        # print(state["end_effector_orientation"])

        # Reshape 9D flattened rotation matrix to 3x3, then convert to euler angles
        rot_matrix = np.array(state["end_effector_orientation"]).reshape(3, 3)
        euler_angles = R.from_matrix(rot_matrix).as_euler('xyz', degrees=False)
        print(euler_angles)


        # print(state["external_wrench"])
        # print(state)
        # q = state["joint_positions"]
        # ee_pos = state["end_effector_position"]

        # 这里先简单打印，你之后可以替换成写入 env、喂 policy 等
        # print("q:", [round(v, 3) for v in q])
        # print("ee:", [round(v, 3) for v in ee_pos])
        # print("-" * 40)

    # def run(self):
    #     """以固定频率循环读取状态。"""
    #     try:
    #         while True:
    #             t0 = time.time()
    #             self.step_once()
    #             elapsed = time.time() - t0
    #             time.sleep(max(0.0, self.dt - elapsed))
    #     except KeyboardInterrupt:
    #         print("Runner stopped by user.")

# if __name__ == "__main__":
#     runner = SimpleRunner(nuc_addr="tcp://192.168.1.7:4242", hz=30.0)
#     runner.run()


if __name__ == "__main__":
    # Strictly align with hz you want to use for testing
    target_hz = 30.0
    runner = SimpleRunner(nuc_addr="tcp://192.168.1.7:4242", hz=target_hz)
    dt = 1.0 / target_hz
    print(f"Target Hz: {target_hz}, dt: {dt:.4f}")
    loop_count = 0
    start_time = time.time()

    while True:
        t0 = time.time()
        runner.step_once()
        loop_count += 1
        elapsed = time.time() - t0

        # Strict sleep so average loop rate matches target hz
        sleep_time = max(0.0, dt - elapsed)
        time.sleep(sleep_time)

        total_time = time.time() - start_time
        actual_hz = loop_count / total_time if total_time > 0 else 0.0
        print(f"[{loop_count}] Actual Hz (avg): {actual_hz:.2f} | Last loop: {1.0/(elapsed+sleep_time):.2f} Hz")

        # You could also print instantaneous hz if you like, comment out if noisy:
        # print(f"Loop {loop_count}: elapsed={elapsed:.4f}s, sleep={sleep_time:.4f}s")

        # Optionally add a break condition for test, e.g. after 100 loops
        # if loop_count >= 100:
        #     break
