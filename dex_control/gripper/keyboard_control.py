'''
Smooth keyboard control for Robotiq gripper.
Hold 's' to close, release to open (like VR trigger). 'q' to quit.

Architecture designed for VR teleop compatibility:
- Release = open (trigger at 0)
- Hold 's' = close (trigger at 1)
- Smooth interpolation between states
'''

import time
import threading
from pynput import keyboard
from robotiq import Robotiq2FGripper


class SmoothGripperController:
    def __init__(self, port='/dev/ttyUSB0', control_rate=30, speed=3.0):
        """
        Args:
            port: Serial port for gripper
            control_rate: Control loop frequency in Hz
            speed: Position change per second when key held (0-255 scale)
        """
        self.gripper = Robotiq2FGripper(port=port)
        self.control_rate = control_rate
        self.speed = speed

        # State
        self.target_pos = float(self.gripper.get_info()[0])
        self.current_pos = self.target_pos
        self.running = False

        # Key states (for hold detection)
        self.keys_pressed = set()

        # Smoothing factor (0-1, higher = more responsive, lower = smoother)
        self.smoothing = 0.3

    def on_press(self, key):
        try:
            self.keys_pressed.add(key.char)
            if key.char == 'q':
                self.running = False
                return False
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            self.keys_pressed.discard(key.char)
        except AttributeError:
            pass

    def control_loop(self):
        """Main control loop running at fixed frequency."""
        dt = 1.0 / self.control_rate
        last_printed_pos = -1

        while self.running:
            loop_start = time.time()

            # VR trigger style: hold 's' = close, release = open
            if 's' in self.keys_pressed:
                self.target_pos = 255  # Closed
            else:
                self.target_pos = 0    # Open

            # Smooth interpolation toward target
            self.current_pos += self.smoothing * (self.target_pos - self.current_pos)

            # Send command to gripper
            pos_int = int(self.current_pos)
            self.gripper.action(position=pos_int, speed=255, force=50)

            # Print only when position changes significantly
            if abs(pos_int - last_printed_pos) >= 1:
                print(f"\rpos: {pos_int:3d}  target: {int(self.target_pos):3d}", end="", flush=True)
                last_printed_pos = pos_int

            # Maintain fixed loop rate
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def run(self):
        print("Smooth Gripper Controller")
        print("=" * 40)
        print("HOLD 's' = close, RELEASE = open, 'q' = quit")
        print(f"Control rate: {self.control_rate} Hz, Speed: {self.speed}/tick")
        print(f"Current position: {int(self.current_pos)}")
        print("=" * 40)

        self.running = True

        # Start control loop in separate thread
        control_thread = threading.Thread(target=self.control_loop, daemon=True)
        control_thread.start()

        # Keyboard listener (blocks until 'q')
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

        print("\nShutting down...")


if __name__ == "__main__":
    controller = SmoothGripperController(
        port='/dev/ttyUSB0',
        control_rate=30,  # 30 Hz control loop
        speed=3.0,        # Position units per tick when key held
    )
    controller.run()
