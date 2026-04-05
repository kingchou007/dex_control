"""Data collection via ZMQ + OpenCV. No ROS2.

Subscribes to robot state from teleop via ZMQ,
captures video directly from camera via OpenCV.

Usage:
    python scripts/data_collect.py
    python scripts/data_collect.py --camera 0
    python scripts/data_collect.py --zmq-port 5557 --host 192.168.1.100
"""

import csv
import h5py
import time
import threading
import sys
import numpy as np
import cv2
import pandas as pd
import click
from datetime import datetime
from pathlib import Path
from collections import deque
from tqdm import tqdm

from dex_control.utils.zmq_publisher import ZMQStateSubscriber

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
TASK_NAME = "gopro_task"
SESSION_TIMESTAMP = datetime.now().strftime("%m.%d-%H:%M")

ROOT_DIR = Path("data_dir/dataset") / TASK_NAME / SESSION_TIMESTAMP
IMG_DIR = ROOT_DIR / "images"
STATE_PATH = ROOT_DIR / "states.csv"
HDF5_PATH = ROOT_DIR / f"data_{SESSION_TIMESTAMP}.hdf5"


# -----------------------------------------------------------------------------
# Camera Capture Thread
# -----------------------------------------------------------------------------
class CameraCapture:
    """Captures video frames from OpenCV VideoCapture in a background thread."""

    def __init__(self, source=0, width=1280, height=720, fps=30):
        self.cap = cv2.VideoCapture(source)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera: {source}")

        self.frame_buffer = deque()
        self.lock = threading.Lock()
        self.is_recording = False
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue
            if self.is_recording:
                ts = time.time()
                with self.lock:
                    self.frame_buffer.append((frame, ts))

    def stop(self):
        self.running = False
        self.thread.join(timeout=2.0)
        self.cap.release()


# -----------------------------------------------------------------------------
# ZMQ State Collector Thread
# -----------------------------------------------------------------------------
class ZMQStateCollector:
    """Collects robot state from ZMQ in a background thread."""

    def __init__(self, host: str = "localhost", port: int = 5557):
        self.subscriber = ZMQStateSubscriber(host=host, port=port)
        self.state_buffer = deque()
        self.lock = threading.Lock()
        self.is_recording = False
        self.running = True
        self.thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.thread.start()

    def _recv_loop(self):
        while self.running:
            state = self.subscriber.receive(timeout=100)
            if state is None:
                continue
            if self.is_recording:
                with self.lock:
                    self.state_buffer.append(state)

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.subscriber.close()


# -----------------------------------------------------------------------------
# Processing
# -----------------------------------------------------------------------------
def process_episode(camera, state_collector, use_gui=True):
    with camera.lock:
        local_video = list(camera.frame_buffer)
    with state_collector.lock:
        local_states = list(state_collector.state_buffer)

    if not local_video:
        print("[Error] No video data.")
        return

    # Build state DataFrame from ZMQ messages
    state_rows = []
    for s in local_states:
        t = s["ee_pose"]["t"]
        q = s["ee_pose"]["q"]
        joints = s.get("joint_positions", [])
        state_rows.append([s["timestamp"], *t, *q, *joints])

    col_names = ['ts', 'PX', 'PY', 'PZ', 'QX', 'QY', 'QZ', 'QW'] + [f'J{i}' for i in range(7)]
    pose_df = pd.DataFrame(state_rows, columns=col_names) if state_rows else pd.DataFrame(columns=col_names)

    data_dict = {'img': [], 'qpos': [], 'action': []}
    pbar = tqdm(range(len(local_video)), desc="Processing") if use_gui else range(len(local_video))

    with open(STATE_PATH, 'a', newline='') as f:
        writer = csv.writer(f)
        for i in pbar:
            frame, frame_ts = local_video[i]
            cv2.imwrite(str(IMG_DIR / f"frame_{i:05d}.jpg"), frame)

            if not pose_df.empty:
                p_idx = (pose_df['ts'] - frame_ts).abs().argmin()
                pose = pose_df.iloc[p_idx][1:8].tolist()  # PX..QW
            else:
                pose = [0.0] * 7

            data_dict['img'].append(frame)
            data_dict['qpos'].append(pose)
            data_dict['action'].append(pose)
            writer.writerow([i, frame_ts, *pose])

    if not use_gui:
        print("[Status] Saving HDF5...")
    with h5py.File(HDF5_PATH, 'w') as h5:
        obs = h5.create_group('observations')
        obs.create_dataset('images/front', data=np.array(data_dict['img'], dtype=np.uint8), compression='gzip')
        obs.create_dataset('qpos', data=np.array(data_dict['qpos'], dtype=np.float32))
        h5.create_dataset('action', data=np.array(data_dict['action'], dtype=np.float32))

    print(f"Saved {len(local_video)} frames, {len(local_states)} states.")


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
@click.command()
@click.option("--camera", default=0, help="Camera device index or URL")
@click.option("--width", default=1280, help="Camera width")
@click.option("--height", default=720, help="Camera height")
@click.option("--fps", default=30, help="Camera FPS")
@click.option("--host", default="localhost", help="ZMQ host (teleop machine)")
@click.option("--zmq-port", default=5557, help="ZMQ port")
@click.option("--no-gui", is_flag=True, help="Disable GUI output")
def main(camera, width, height, fps, host, zmq_port, no_gui):
    """Collect data: video from camera + robot state from ZMQ."""

    # Setup directories
    for d in [ROOT_DIR, IMG_DIR]:
        d.mkdir(parents=True, exist_ok=True)

    # Initialize
    print(f"[Session: {SESSION_TIMESTAMP}]")
    print(f"Camera: {camera} ({width}x{height} @ {fps}fps)")
    print(f"ZMQ: {host}:{zmq_port}")

    cam = CameraCapture(source=camera, width=width, height=height, fps=fps)
    state_col = ZMQStateCollector(host=host, port=zmq_port)

    if not STATE_PATH.exists():
        with open(STATE_PATH, 'w', newline='') as f:
            csv.writer(f).writerow(['Idx', 'TS', 'PX', 'PY', 'PZ', 'QX', 'QY', 'QZ', 'QW'])

    try:
        input("\nPress Enter to START recording...")

        cam.is_recording = True
        state_col.is_recording = True
        input("RECORDING... Press Enter to STOP.\n")
        cam.is_recording = False
        state_col.is_recording = False

        process_episode(cam, state_col, use_gui=not no_gui)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        cam.stop()
        state_col.stop()


if __name__ == "__main__":
    main()
