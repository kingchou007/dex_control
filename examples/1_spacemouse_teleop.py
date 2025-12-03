"""SpaceMouse teleoperation for Franka robot with gripper control

Supports translation control and binary gripper control (grasp/release).

Usage:
    python scripts/spacemouse_teleop.py [OPTIONS]

Options:
    --ip TEXT                 Robot server IP address [default: 192.168.1.7]
    --port INTEGER            Robot server port [default: 4242]
    --vendor-id INTEGER       SpaceMouse vendor ID [default: 9583]
    --product-id INTEGER      SpaceMouse product ID [default: 50741]
    --translation-scale FLOAT Translation velocity scale [default: 1.0]
    --control-rate INTEGER    Control rate in Hz [default: 50]
    --help                    Show this message and exit

Controls:
    - Move SpaceMouse: Control robot end-effector translation
    - Left Button: Toggle gripper (grasp/release)
    - Ctrl+C: Exit

Examples:
    # Use default settings
    python examples/1_spacemouse_teleop.py

    # Connect to different robot IP
    python examples/1_spacemouse_teleop.py --ip 192.168.1.7

Reference: https://github.com/UT-Austin-RPL/deoxys_control/blob/main/deoxys/deoxys/utils/io_devices/spacemouse.py

#TODO: use multiprocessing to replace the threading
"""



import threading
import time
import hid
import click
from termcolor import cprint

# sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from dex_control.robot.robot_client import FrankaRobotClient

# Default CONFIG
DEFAULT_PRODUCT_ID = 50741  # Wireless=50735/50741, Wired=50770
DEFAULT_VENDOR_ID = 9583
DEFAULT_IP = "192.168.1.7"
DEFAULT_PORT = 4242

# Control parameters
TRANSLATION_VELOCITY_SCALE = 1.0  # m/s per unit
CONTROL_RATE_HZ = 50
MAX_DELTA = 0.5  # Maximum position delta per step (m)

# HELPERS
def to_int16(y1, y2):
    """Convert two 8-bit bytes to signed 16-bit integer"""
    x = (y1) | (y2 << 8)
    if x >= 32768:
        x = -(65536 - x)
    return x

def scale(x, scale=350.0):
    """Scale raw HID value to normalized range"""
    return x / scale

def convert(b1, b2):
    """Convert HID bytes to scaled value"""
    return scale(to_int16(b1, b2))

def clamp(value, min_val, max_val):
    """Clamp value between min and max"""
    return max(min_val, min(max_val, value))

class SpaceMouse:
    """SpaceMouse driver with button support"""
    
    def __init__(self, vendor_id, product_id):
        self.vendor_id = vendor_id
        self.product_id = product_id
        cprint(f"Opening SpaceMouse (VID={vendor_id}, PID={product_id})...", "cyan")
        try:
            self.device = hid.device()
            self.device.open(self.vendor_id, self.product_id)
            self.device.set_nonblocking(1)
            cprint("SpaceMouse connected!", "green")
        except Exception as e:
            cprint(f"Error: {e}", "red")
            exit(1)

        # 6-DOF data: [x, y, z, roll, pitch, yaw]
        self.data = [0.0] * 6
        
        # Button states
        self.left_button_pressed = False
        self.right_button_pressed = False
        self.button_changed = False
        
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _read_loop(self):
        """Background thread to read HID data"""
        while self.running:
            try:
                d = self.device.read(13)
                if not d:
                    continue
                    
                if d[0] == 1:  # Translation packet
                    self.data[0] = convert(d[3], d[4])  # X
                    self.data[1] = convert(d[1], d[2])  # Y
                    self.data[2] = convert(d[5], d[6]) * -1.0  # Z
                    
                    # Some models send rotation in same packet
                    if len(d) > 12:
                        self.data[3] = convert(d[7], d[8])
                        self.data[4] = convert(d[9], d[10])
                        self.data[5] = convert(d[11], d[12])

                elif d[0] == 2:  # Rotation packet
                    if len(d) >= 7:
                        self.data[3] = convert(d[1], d[2])
                        self.data[4] = convert(d[3], d[4])
                        self.data[5] = convert(d[5], d[6])
                
                elif d[0] == 3:  # Button packet
                    if len(d) >= 2:
                        old_left = self.left_button_pressed
                        old_right = self.right_button_pressed
                        
                        # Left button (d[1] == 1 when pressed)
                        self.left_button_pressed = (d[1] == 1)
                        # Right button (d[1] == 2 when pressed)
                        self.right_button_pressed = (d[1] == 2)
                        
                        # Detect button state change
                        if old_left != self.left_button_pressed or old_right != self.right_button_pressed:
                            self.button_changed = True
                            
            except:
                pass

    def get_data(self):
        """Get current 6-DOF data"""
        return self.data.copy()
    
    def get_button_state(self):
        """Get button states and reset changed flag"""
        state = {
            'left': self.left_button_pressed,
            'right': self.right_button_pressed,
            'changed': self.button_changed
        }
        self.button_changed = False
        return state

@click.command()
@click.option('--ip', default=DEFAULT_IP, help='Robot server IP address')
@click.option('--port', default=DEFAULT_PORT, help='Robot server port')
@click.option('--vendor-id', default=DEFAULT_VENDOR_ID, help='SpaceMouse vendor ID')
@click.option('--product-id', default=DEFAULT_PRODUCT_ID, help='SpaceMouse product ID')
@click.option('--translation-scale', default=TRANSLATION_VELOCITY_SCALE, help='Translation velocity scale')
@click.option('--control-rate', default=CONTROL_RATE_HZ, help='Control rate in Hz')
def main(ip, port, vendor_id, product_id, translation_scale, control_rate):
    """SpaceMouse teleoperation for Franka robot"""
    
    # Build server address
    server_addr = f"tcp://{ip}:{port}"
    
    # Connect to robot
    try:
        cprint(f"Connecting to robot at {server_addr}...", "cyan")
        robot = FrankaRobotClient(server_addr=server_addr)
        cprint("Robot connected!", "green")
    except Exception as e:
        cprint(f"Error: {e}", "red")
        return
    
    # Initialize SpaceMouse
    spacemouse = SpaceMouse(vendor_id, product_id)
    dt = 1.0 / control_rate
    
    # State tracking
    error_count = 0
    skip_commands = 0
    gripper_state = False  # False = closed, True = open

    cprint("\n" + "="*60, "yellow")
    cprint("SpaceMouse Teleoperation Active", "yellow", attrs=["bold"])
    cprint("="*60, "yellow")
    cprint("Controls:", "cyan")
    cprint("  - Move SpaceMouse: Control robot end-effector", "white")
    cprint("  - Left Button: Toggle gripper (grasp/release)", "white")
    cprint("  - Ctrl+C: Exit", "white")
    cprint("="*60 + "\n", "yellow")
    
    try:
        while True:
            # Get SpaceMouse data
            raw = spacemouse.get_data()
            buttons = spacemouse.get_button_state()
            
            # Handle button press (gripper control - binary grasp/release)
            if buttons['changed'] and buttons['left']:
                gripper_state = not gripper_state
                try:
                    if gripper_state:
                        robot.release_object()
                        cprint("Released gripper", "green")
                    else:
                        robot.grasp_object()
                        cprint("Grasping object", "green")
                except Exception as e:
                    cprint(f"Gripper error: {e}", "red")
            
            # Skip if in error recovery
            if skip_commands > 0:
                skip_commands -= 1
                time.sleep(dt)
                continue

            # Calculate position delta
            vel_x = raw[0] * translation_scale
            vel_y = raw[1] * translation_scale
            vel_z = raw[2] * translation_scale
            
            delta_x = clamp(vel_x * dt, -MAX_DELTA, MAX_DELTA)
            delta_y = clamp(vel_y * dt, -MAX_DELTA, MAX_DELTA)
            delta_z = clamp(vel_z * dt, -MAX_DELTA, MAX_DELTA)

            # Send command if significant movement
            if any(abs(v) > 0.005 for v in [delta_x, delta_y, delta_z]):
                try:
                    robot.move_ee_pose(
                        [delta_x, delta_y, delta_z],
                        asynchronous=True,
                        delta=True
                    )
                    if error_count > 0:
                        error_count -= 1
                        
                except Exception as e:
                    error_msg = str(e)
                    if "Reflex" in error_msg or "aborted" in error_msg:
                        error_count += 1
                        if error_count == 1:
                            cprint("\nRobot error - pausing...", "yellow")
                        skip_commands = 25
                        time.sleep(1.0)
                    else:
                        cprint(f"Error: {e}", "red")

            time.sleep(dt)
            
    except KeyboardInterrupt:
        cprint("\n\nShutting down...", "yellow")
        spacemouse.running = False



if __name__ == "__main__":
    main()