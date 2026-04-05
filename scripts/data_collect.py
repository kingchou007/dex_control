import os
import csv
import h5py
import time
import threading
import sys
import numpy as np
import cv2
import pandas as pd
import soundfile as sf
from datetime import datetime
from pathlib import Path
from collections import deque
from tqdm import tqdm

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from sounddevice_ros.msg import AudioInfo, AudioData

# -----------------------------------------------------------------------------
# Configuration & Argument Parsing
# -----------------------------------------------------------------------------
TASK_NAME = "gopro_task"
SESSION_TIMESTAMP = datetime.now().strftime("%m.%d-%H:%M")

# Flags
USE_AUDIO = "--no_audio" not in sys.argv
USE_GUI = "--no_gui" not in sys.argv

ROOT_DIR = Path("data_dir/dataset") / TASK_NAME / SESSION_TIMESTAMP
IMG_DIR = ROOT_DIR / "images"
AUD_DIR = ROOT_DIR / "audio"
STATE_PATH = ROOT_DIR / "states.csv"
HDF5_PATH = ROOT_DIR / f"data_{SESSION_TIMESTAMP}.hdf5"
WAV_PATH = AUD_DIR / f"audio_{SESSION_TIMESTAMP}.wav"

# Setup Directories
dirs_to_make = [ROOT_DIR, IMG_DIR]
if USE_AUDIO: dirs_to_make.append(AUD_DIR)
for d in dirs_to_make:
    d.mkdir(parents=True, exist_ok=True)

VIDEO_TOPIC = "/gopro/image_raw/compressed"
EE_POSE_TOPIC = "/robot/ee_pose"        
TARGET_ACTION_TOPIC = "/robot/target_action"
AUDIO_DATA_TOPIC = "/audio"
AUDIO_INFO_TOPIC = "/audio_info"

SAMPLE_RATE_HZ = 5.0 
SAMPLE_PERIOD = 1.0 / SAMPLE_RATE_HZ 

# -----------------------------------------------------------------------------
# ROS Node
# -----------------------------------------------------------------------------
class PairedDataNode(Node):
    def __init__(self):
        super().__init__('paired_data_recorder')

        self.video_buffer = deque()
        self.ee_pose_buffer = deque()
        self.target_action_buffer = deque()
        
        self.buffer_lock = threading.Lock()
        self.is_recording = False

        self.audio_file = None
        self.sample_rate = 44100
        self.num_channels = 1
        self.subtype = 'FLOAT'
        self.audio_info_received = threading.Event()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(CompressedImage, VIDEO_TOPIC, self._video_callback, qos)
        self.create_subscription(PoseStamped, EE_POSE_TOPIC, self._ee_pose_callback, qos)
        self.create_subscription(PoseStamped, TARGET_ACTION_TOPIC, self._target_action_callback, qos)
        
        if USE_AUDIO:
            self.create_subscription(AudioData, AUDIO_DATA_TOPIC, self._audio_callback, qos)
            self.create_subscription(AudioInfo, AUDIO_INFO_TOPIC, self._info_callback, 10)

    def _info_callback(self, msg):
        if not self.audio_info_received.is_set():
            self.sample_rate = msg.sample_rate
            self.num_channels = msg.num_channels
            if msg.subtype: self.subtype = msg.subtype
            self.audio_info_received.set()
            if USE_GUI: self.get_logger().info(f"Audio Configured: {self.sample_rate}Hz")

    def _audio_callback(self, msg):
        if not self.is_recording or self.audio_file is None: return
        try:
            audio_data = np.asarray(msg.data).reshape((-1, self.num_channels))
            self.audio_file.write(audio_data)
        except Exception:
            pass

    def _video_callback(self, msg):
        if not self.is_recording: return
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        with self.buffer_lock:
            self.video_buffer.append((frame, ts))

    def _ee_pose_callback(self, msg):
        if not self.is_recording: return
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose_arr = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        with self.buffer_lock:
            self.ee_pose_buffer.append((ts, *pose_arr))

    def _target_action_callback(self, msg):
        if not self.is_recording: return
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        action_arr = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                      msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        with self.buffer_lock:
            self.target_action_buffer.append((ts, *action_arr))

# -----------------------------------------------------------------------------
# Processing Logic
# -----------------------------------------------------------------------------
def process_episode(node):
    with node.buffer_lock:
        local_video = list(node.video_buffer)
        local_pose = list(node.ee_pose_buffer)
        local_action = list(node.target_action_buffer)

    if not local_video:
        print("[Error] No data to process.")
        return

    pose_df = pd.DataFrame(local_pose, columns=['ts', 'PX', 'PY', 'PZ', 'QX', 'QY', 'QZ', 'QW'])
    action_df = pd.DataFrame(local_action, columns=['ts', 'AX', 'AY', 'AZ', 'AQX', 'AQY', 'AQZ', 'AQW'])
    
    data_dict = {'img': [], 'qpos': [], 'action': []}

    # Conditional tqdm based on USE_GUI
    pbar = tqdm(range(len(local_video)), desc="Processing") if USE_GUI else range(len(local_video))

    with open(STATE_PATH, 'a', newline='') as f:
        writer = csv.writer(f)
        for i in pbar:
            frame, frame_ts = local_video[i]
            cv2.imwrite(str(IMG_DIR / f"frame_{i:05d}.jpg"), frame)

            p_idx = (pose_df['ts'] - frame_ts).abs().argmin() if not pose_df.empty else None
            pose = pose_df.iloc[p_idx][1:].tolist() if p_idx is not None else [0.0]*7
            
            a_idx = (action_df['ts'] - frame_ts).abs().argmin() if not action_df.empty else None
            action = action_df.iloc[a_idx][1:].tolist() if a_idx is not None else [0.0]*7

            data_dict['img'].append(frame)
            data_dict['qpos'].append(pose)
            data_dict['action'].append(action)
            writer.writerow([i, frame_ts, *pose, *action])

    if not USE_GUI: print("[Status] Saving HDF5...")
    with h5py.File(HDF5_PATH, 'w') as h5:
        obs = h5.create_group('observations')
        obs.create_dataset('images/front', data=np.array(data_dict['img'], dtype=np.uint8), compression='gzip')
        obs.create_dataset('qpos', data=np.array(data_dict['qpos'], dtype=np.float32))
        h5.create_dataset('action', data=np.array(data_dict['action'], dtype=np.float32))

# -----------------------------------------------------------------------------
# Main Loop
# -----------------------------------------------------------------------------
def main():
    rclpy.init()
    node = PairedDataNode()

    if not STATE_PATH.exists():
        with open(STATE_PATH, 'w', newline='') as f:
            csv.writer(f).writerow(['Idx', 'TS', 'PX', 'PY', 'PZ', 'QX', 'QY', 'QZ', 'QW', 'AX', 'AY', 'AZ', 'AQX', 'AQY', 'AQZ', 'AQW'])

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    try:
        if USE_AUDIO:
            if USE_GUI: print(f"\n[Session: {SESSION_TIMESTAMP}] Waiting for Audio Info...")
            while rclpy.ok() and not node.audio_info_received.wait(timeout=0.1):
                pass
        
        input("Press Enter to START recording...")
        
        if USE_AUDIO:
            with sf.SoundFile(str(WAV_PATH), mode='x', 
                              samplerate=node.sample_rate, 
                              channels=node.num_channels, 
                              subtype=node.subtype) as node.audio_file:
                node.is_recording = True
                input("RECORDING... Press Enter to STOP.")
                node.is_recording = False
            node.audio_file = None
        else:
            node.is_recording = True
            input("RECORDING (No Audio)... Press Enter to STOP.")
            node.is_recording = False
        
        process_episode(node)
        if USE_GUI: print(f"Processing Complete.")

    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()