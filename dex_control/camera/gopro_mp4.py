import sys
import os
import time
import threading
import pathlib
from typing import Optional, Tuple

import click
import cv2
import numpy as np

# -----------------------------------------------------------------------------
# Environment Setup
# -----------------------------------------------------------------------------
def _configure_display_env(default_display: str = ":0") -> None:
    """Configures the display environment variables for GUI support."""
    display_arg = None
    argv = sys.argv
    for i, arg in enumerate(argv):
        if arg.startswith('--display='):
            display_arg = arg.split('=', 1)[1]
        elif arg == '--display' and i + 1 < len(argv):
            display_arg = argv[i + 1]

    if display_arg:
        os.environ['DISPLAY'] = display_arg
    elif not os.environ.get('DISPLAY'):
        os.environ['DISPLAY'] = default_display

    os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')

_configure_display_env()

# -----------------------------------------------------------------------------
# Fast Video Capture Class (MJPEG Force)
# -----------------------------------------------------------------------------
class FastVideoCapture:
    def __init__(self, src: str, width: int, height: int, fps: int):
        self.src = src
        self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2)

        # Enforce MJPEG for high framerate
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.grabbed, self.frame = self.cap.read()
        if not self.grabbed:
            raise RuntimeError(f"FastVideoCapture: Unable to read device {src}")

        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        print(f"[Camera] Initialized: Requested {width}x{height}@{fps}FPS -> "
              f"Actual {int(actual_w)}x{int(actual_h)}@{actual_fps:.1f}FPS")

        self.started = False
        self.read_lock = threading.Lock()
        self.thread: Optional[threading.Thread] = None

    def start(self):
        if self.started:
            return self
        self.started = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        return self

    def _update(self):
        while self.started:
            grabbed, frame = self.cap.read()
            with self.read_lock:
                self.grabbed = grabbed
                self.frame = frame
            # Small sleep to prevent CPU hogging
            time.sleep(0.0005)

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        with self.read_lock:
            if self.frame is not None:
                return self.grabbed, self.frame.copy()
            return self.grabbed, None

    def stop(self):
        self.started = False
        if self.thread:
            self.thread.join()
        self.cap.release()

# -----------------------------------------------------------------------------
# Helper Functions
# -----------------------------------------------------------------------------
def get_sorted_v4l_paths(by_id: bool = True) -> list:
    """Finds valid V4L2 device paths."""
    dirname = 'by-id' if by_id else 'by-path'
    v4l_dir = pathlib.Path('/dev/v4l').joinpath(dirname)
    valid_paths = []

    # Safely handle if directory doesn't exist
    if not v4l_dir.exists():
        return []

    for dev_path in sorted(v4l_dir.glob("*video*")):
        name = dev_path.name
        if "index0" in name:
            valid_paths.append(str(dev_path.absolute()))
    return valid_paths

# -----------------------------------------------------------------------------
# Main Execution
# -----------------------------------------------------------------------------
@click.command()
@click.option('--camera_idx', default=0, type=int, help="Index of V4L2 device if path not provided")
@click.option('--dev_path', default=None, help="Specific device path (e.g., /dev/video0)")
@click.option('--display', default=None, help="X11 Display")
@click.option('--no_gui', is_flag=True, help="Disable local video preview")
def main(camera_idx, dev_path, display, no_gui):

    # 1. Setup Camera
    v4l_paths = get_sorted_v4l_paths()
    if dev_path:
        dev_path = str(pathlib.Path(dev_path))
    else:
        if not v4l_paths:
            raise RuntimeError("No V4L2 devices found.")
        dev_path = v4l_paths[camera_idx]

    target_res = (1920, 1080)
    target_fps = 60

    print(f"Opening {dev_path} @ {target_res} {target_fps}FPS (MJPEG Force)...")

    try:
        cap_thread = FastVideoCapture(dev_path, target_res[0], target_res[1], target_fps)
        cap_thread.start()
    except Exception as e:
        print(f"[ERROR] Camera failed: {e}")
        return

    print("Streaming (1080p60)... Press 'q' to stop.")

    last_vis_time = 0
    vis_interval = 1.0 / 30.0  # Limit GUI updates to 30 FPS

    try:
        while True:
            ret, frame = cap_thread.read()
            if ret and frame is not None:
                # --- GUI Visualization ---
                if not no_gui:
                    now = time.time()
                    if (now - last_vis_time) > vis_interval:
                        last_vis_time = now
                        h, w = frame.shape[:2]
                        scale = 0.5
                        preview = cv2.resize(frame, (int(w * scale), int(h * scale)),
                                           interpolation=cv2.INTER_NEAREST)
                        cv2.imshow('camera', preview)

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
            else:
                time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        cap_thread.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
