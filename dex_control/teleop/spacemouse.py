import threading
import time
from collections import namedtuple
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from dex_control.teleop.controller import Controller


def unit_vector(data, axis=None, out=None):
    """
    Returns ndarray normalized by length, i.e. eucledian norm, along axis.

    Examples:

        >>> v0 = numpy.random.random(3)
        >>> v1 = unit_vector(v0)
        >>> numpy.allclose(v1, v0 / numpy.linalg.norm(v0))
        True
        >>> v0 = numpy.random.rand(5, 4, 3)
        >>> v1 = unit_vector(v0, axis=-1)
        >>> v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0*v0, axis=2)), 2)
        >>> numpy.allclose(v1, v2)
        True
        >>> v1 = unit_vector(v0, axis=1)
        >>> v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0*v0, axis=1)), 1)
        >>> numpy.allclose(v1, v2)
        True
        >>> v1 = numpy.empty((5, 4, 3), dtype=numpy.float32)
        >>> unit_vector(v0, axis=1, out=v1)
        >>> numpy.allclose(v1, v2)
        True
        >>> list(unit_vector([]))
        []
        >>> list(unit_vector([1.0]))
        [1.0]

    """
    if out is None:
        data = np.array(data, dtype=np.float32, copy=True)
        if data.ndim == 1:
            data /= math.sqrt(np.dot(data, data))
            return data
    else:
        if out is not data:
            out[:] = np.array(data, copy=False)
        data = out
    length = np.atleast_1d(np.sum(data * data, axis))
    np.sqrt(length, length)
    if axis is not None:
        length = np.expand_dims(length, axis)
    data /= length
    if out is None:
        return data


def rotation_matrix(angle, direction, point=None):
    """
    Returns matrix to rotate about axis defined by point and direction.

    Examples:

        >>> angle = (random.random() - 0.5) * (2*math.pi)
        >>> direc = numpy.random.random(3) - 0.5
        >>> point = numpy.random.random(3) - 0.5
        >>> R0 = rotation_matrix(angle, direc, point)
        >>> R1 = rotation_matrix(angle-2*math.pi, direc, point)
        >>> is_same_transform(R0, R1)
        True
        >>> R0 = rotation_matrix(angle, direc, point)
        >>> R1 = rotation_matrix(-angle, -direc, point)
        >>> is_same_transform(R0, R1)
        True
        >>> I = numpy.identity(4, numpy.float32)
        >>> numpy.allclose(I, rotation_matrix(math.pi*2, direc))
        True
        >>> numpy.allclose(2., numpy.trace(rotation_matrix(math.pi/2,
        ...                                                direc, point)))
        True

    """
    sina = math.sin(angle)
    cosa = math.cos(angle)
    direction = unit_vector(direction[:3])
    # rotation matrix around unit vector
    R = np.array(((cosa, 0.0, 0.0), (0.0, cosa, 0.0), (0.0, 0.0, cosa)), dtype=np.float32)
    R += np.outer(direction, direction) * (1.0 - cosa)
    direction *= sina
    R += np.array(
        (
            (0.0, -direction[2], direction[1]),
            (direction[2], 0.0, -direction[0]),
            (-direction[1], direction[0], 0.0),
        ),
        dtype=np.float32,
    )
    M = np.identity(4)
    M[:3, :3] = R
    if point is not None:
        # rotation not around origin
        point = np.array(point[:3], dtype=np.float32, copy=False)
        M[:3, 3] = point - np.dot(R, point)
    return M


try:
    import os

    os.environ["LD_LIBRARY_PATH"] = os.getcwd()  # or whatever path you want
    import hid
except ModuleNotFoundError as exc:
    raise ImportError(
        "Unable to load module hid, please check if SpaceMouse is connected. \n"
        "Type 'lsusb' and compare pid with 0x256f:0xc635. \n"
    ) from exc


AxisSpec = namedtuple("AxisSpec", ["channel", "byte1", "byte2", "scale"])

SPACE_MOUSE_SPEC = {
    "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
    "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
    "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
    "roll": AxisSpec(channel=1, byte1=7, byte2=8, scale=-1),
    "pitch": AxisSpec(channel=1, byte1=9, byte2=10, scale=-1),
    "yaw": AxisSpec(channel=1, byte1=11, byte2=12, scale=1),
}


def to_int16(y1, y2):
    """
    Convert two 8 bit bytes to a signed 16 bit integer.
    Args:
        y1 (int): 8-bit byte
        y2 (int): 8-bit byte
    Returns:
        int: 16-bit integer
    """
    x = (y1) | (y2 << 8)
    if x >= 32768:
        x = -(65536 - x)
    return x


def scale_to_control(x, axis_scale=350.0, min_v=-1.0, max_v=1.0):
    """
    Normalize raw HID readings to target range.
    Args:
        x (int): Raw reading from HID
        axis_scale (float): (Inverted) scaling factor for mapping raw input value
        min_v (float): Minimum limit after scaling
        max_v (float): Maximum limit after scaling
    Returns:
        float: Clipped, scaled input from HID
    """
    x = x / axis_scale
    x = min(max(x, min_v), max_v)
    return x


def convert(b1, b2):
    """
    Converts SpaceMouse message to commands.
    Args:
        b1 (int): 8-bit byte
        b2 (int): 8-bit byte
    Returns:
        float: Scaled value from Spacemouse message
    """
    return scale_to_control(to_int16(b1, b2))


class SpaceMouseInterface:
    """
    A minimalistic driver class for SpaceMouse with HID library.
    Note: Use hid.enumerate() to view all USB human interface devices (HID).
    Make sure SpaceMouse is detected before running the script.
    You can look up its vendor/product id from this method.
    Args:
        vendor_id (int): HID device vendor id
        product_id (int): HID device product id
        pos_sensitivity (float): Magnitude of input position command scaling
        rot_sensitivity (float): Magnitude of scale input rotation commands scaling

    See https://zhuyifengzju.github.io/deoxys_docs/html/tutorials/using_teleoperation_devices.html
    """

    def __init__(
        self,
        vendor_id=0x256f,  # Bus 003 Device 043
        product_id=0xc635, # 3Dconnexion SpaceMouse Compact
        pos_sensitivity=5,
        rot_sensitivity=5,
        action_scale=0.08,
        deadzone=0.05,  # Added deadzone
        smoothing=0.5,  # Added smoothing factor
    ):
        print("Opening SpaceMouse device")
        # print(hid.enumerate())
        # print(vendor_id, product_id)
        self.device = hid.device()
        self.device.open(vendor_id, product_id)  # SpaceMouse

        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity
        self.action_scale = action_scale
        self.deadzone = deadzone
        self.smoothing = smoothing
        
        # Previous values for smoothing
        self._prev_dpos = np.zeros(3)
        self._prev_drot = np.zeros(3)

        self.gripper_is_closed = False
        print("Manufacturer: %s" % self.device.get_manufacturer_string())
        print("Product: %s" % self.device.get_product_string())

        # 6-DOF variables
        self.x, self.y, self.z = 0, 0, 0
        self.roll, self.pitch, self.yaw = 0, 0, 0

        self._display_controls()

        self.single_click_and_hold = False
        self.elapsed_time = 0

        self._control = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.lock_state = 0
        self.rotation = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])

        # Launch listener thread
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()

    @staticmethod
    def _display_controls():
        """
        Method to pretty print controls. 
        """

        def print_command(char, info):
            char += " " * (30 - len(char))
            print("{}\t{}".format(char, info))

        print("")
        print_command("Control", "Command")
        print_command("Right button", "reset simulation")
        print_command("Left button (hold)", "toggle gripper")
        print_command("Move mouse laterally", "move arm horizontally in x-y plane")
        print_command("Move mouse vertically", "move arm vertically")
        print_command("Twist mouse about an axis", "rotate arm about a corresponding axis")
        print("")

    def _reset_internal_state(self):
        """
        Resets internal state of controller, except for the reset signal.
        """
        self.rotation = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])

        self.x, self.y, self.z = 0, 0, 0
        self.roll, self.pitch, self.yaw = 0, 0, 0

        self._control = np.zeros(6)

        self.single_click_and_hold = False
        self.t_last_click = time.time()

    def start_control(self):
        """
        Method that should be called externally before controller can
        start receiving commands.
        """
        self._reset_internal_state()
        self.lock_state = 0

    def get_controller_state(self):
        """
        Grabs the current state of the 3D mouse.
        Returns:
            dict: A dictionary containing dpos, orn, unmodified orn, grasp, and reset
        """
        raw_dpos = np.array(self.control[:3]) * self.action_scale
        raw_drot = np.array(self.control[3:]) * self.action_scale
        
        # Apply better mapping for intuitive control
        dpos = np.array([-raw_dpos[1], raw_dpos[0], -raw_dpos[2]])
        drot = np.array([-raw_drot[1], raw_drot[0], raw_drot[2]])
        
        # Apply deadzone
        dpos = self._apply_deadzone(dpos, self.deadzone)
        drot = self._apply_deadzone(drot, self.deadzone)
        
        # Apply smoothing
        dpos = self._smooth_control(dpos, self._prev_dpos, self.smoothing)
        drot = self._smooth_control(drot, self._prev_drot, self.smoothing)
        
        # Store for next smoothing cycle
        self._prev_dpos = dpos
        self._prev_drot = drot
        
        # Handle rotation matrix calculation for visualization if needed
        roll, pitch, yaw = raw_drot
        drot1 = rotation_matrix(angle=-pitch, direction=[1.0, 0, 0], point=None)[:3, :3]
        drot2 = rotation_matrix(angle=roll, direction=[0, 1.0, 0], point=None)[:3, :3]
        drot3 = rotation_matrix(angle=yaw, direction=[0, 0, 1.0], point=None)[:3, :3]
        self.rotation = self.rotation.dot(drot1.dot(drot2.dot(drot3)))

        return dict(
            dpos=dpos,  # Remapped position delta
            rotation=self.rotation,
            raw_drotation=drot,  # Remapped rotation delta
            grasp=self.control_gripper,
            hold=self.single_click_and_hold,
            lock=self.lock_state,
        )
    
    def _apply_deadzone(self, values, threshold):
        """Apply deadzone filter to control values"""
        magnitude = np.linalg.norm(values)
        if magnitude < threshold:
            return np.zeros_like(values)
        else:
            # Scale values to maintain continuity at deadzone boundary
            scale = (magnitude - threshold) / (magnitude * (1 - threshold))
            return values * scale
    
    def _smooth_control(self, current, previous, smoothing_factor):
        """Apply exponential smoothing to control values"""
        return smoothing_factor * previous + (1 - smoothing_factor) * current
    
    def _apply_response_curve(self, values, curve_factor=2.0):
        """Apply exponential response curve for better fine control"""
        signs = np.sign(values)
        magnitudes = np.abs(values)
        return signs * (magnitudes ** curve_factor)

    def run(self):
        """Listener method that keeps pulling new messages."""
        last_debug_time = time.time()
        debug_interval = 0.5  # Debug output at 2Hz
        
        while True:
            try:
                d = self.device.read(13)  # Read more bytes for complete data
                if d is None or len(d) < 2:
                    continue

                # Throttled debug output
                if hasattr(self, 'debug') and self.debug:
                    current_time = time.time()
                    if current_time - last_debug_time > debug_interval:
                        last_debug_time = current_time
                        print(f"Raw data: {d}")
                        print(f"Control: {self._control}")
                        print(f"Button states - Grip: {self.gripper_is_closed}, Hold: {self.single_click_and_hold}, Reset: {self.lock_state}")

                # Translation data (X, Y, Z movement)
                if d[0] == 1:
                    x = convert(d[1], d[2]) if len(d) > 2 else 0.0
                    y = convert(d[3], d[4]) if len(d) > 4 else 0.0
                    z = convert(d[5], d[6]) if len(d) > 6 else 0.0
                    
                    # Scale and store translation values with sensitivity
                    self._control[0] = x * self.pos_sensitivity  
                    self._control[1] = y * self.pos_sensitivity
                    self._control[2] = z * self.pos_sensitivity
                    
                # Rotation data (Roll, Pitch, Yaw)
                elif d[0] == 2:
                    roll = convert(d[1], d[2]) if len(d) > 2 else 0.0
                    pitch = convert(d[3], d[4]) if len(d) > 4 else 0.0
                    yaw = convert(d[5], d[6]) if len(d) > 6 else 0.0
                    
                    # Store rotation values directly, no complex matrix operations
                    self._control[3] = roll * self.rot_sensitivity
                    self._control[4] = pitch * self.rot_sensitivity
                    self._control[5] = yaw * self.rot_sensitivity
                    
                # Button data packet
                elif d[0] == 3:  # Button data on SpaceMouse Compact
                    if len(d) >= 2:
                        button_state = d[1]
                        
                        # Left button - toggle gripper
                        if button_state & 1:  # Bit 0 set
                            t_click = time.time()
                            if not hasattr(self, 't_last_click'):
                                self.t_last_click = t_click
                            self.elapsed_time = t_click - self.t_last_click
                            self.t_last_click = t_click
                            
                            # Toggle gripper state on press
                            if self.elapsed_time > 0.5:
                                self.gripper_is_closed = not self.gripper_is_closed
                                print(f"Gripper state changed: {'closed' if self.gripper_is_closed else 'open'}")
                            
                            self.single_click_and_hold = True
                        else:
                            self.single_click_and_hold = False
                        
                        # Right button - reset
                        if button_state & 2:  # Bit 1 set
                            self.lock_state = 1
                            print("Reset triggered by spacemouse button")
                        else:
                            self.lock_state = 0

            except Exception as e:
                print(f"SpaceMouse error: {e}")
                self._control = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                continue

    @property
    def control(self):
        """
        Grabs current pose of Spacemouse
        Returns:
            np.array: 6-DoF control value
        """

        return np.array(self._control)

    @property
    def control_gripper(self):
        """
        Maps internal states into gripper commands.
        Returns:
            float: Whether we're using single click and hold or not
        """
        return self.gripper_is_closed

    def get_action(self):
        if sum(abs(self.control)) > 0.0 or self.control_gripper is not None:
            return (
                self.action_scale * self.control,
                self.control_gripper,
                self.lock_state,
            )
        else:
            return None, self.control_gripper, self.lock_state

    def debug_mode(self, enable=True):
        """Enable debug mode to print all raw inputs from the device"""
        self.debug = enable
        print(f"Debug mode {'enabled' if enable else 'disabled'}")
        
        # Dump current state
        if enable:
            print("\nCURRENT STATE:")
            print(f"Position control: {self._control[:3]}")
            print(f"Rotation control: {self._control[3:]}")
            print(f"Gripper is closed: {self.gripper_is_closed} (returns {int(self.gripper_is_closed)})")
            print(f"Current grasp command: {self.control_gripper} (returned to teleop)")
            print(f"Single click hold: {self.single_click_and_hold}")
            print(f"Lock state: {self.lock_state}")
            print("\nWAITING FOR INPUT - move mouse or press buttons...")
            
        # Capture and print a few raw packets to understand the data structure
        if enable:
            for i in range(5):
                d = self.device.read(13)
                if d is not None:
                    print(f"Packet {i}: {d}")
                time.sleep(0.1)


class SpaceMouse(Controller):
    def __init__(
        self,
        max_lin_vel: float = 3,
        max_rot_vel: float = 3,
        max_gripper_vel: float = 3,
        pos_sensitivity: float = 8.0,  # Adjusted
        rot_sensitivity: float = 8.0,  # Adjusted
        action_scale: float = 0.1,
        deadzone: float = 0.05,       # Added
        smoothing: float = 0.3        # Added
    ):
        self.action_space = "cartesian_velocity"
        self.gripper_action_space = "velocity"
        self.max_lin_vel = max_lin_vel
        self.max_rot_vel = max_rot_vel
        self.max_gripper_vel = max_gripper_vel
        
        # Create SpaceMouseInterface with improved parameters
        self.interface = SpaceMouseInterface(
            pos_sensitivity=pos_sensitivity,
            rot_sensitivity=rot_sensitivity,
            action_scale=action_scale,
            deadzone=deadzone,
            smoothing=smoothing
        )
        
        # Internal state
        self._state = {
            "success": False,
            "failure": False,
            "movement_enabled": True,
            "controller_on": True,
        }
        
        print("\nSpaceMouse controls:")
        print("- Move to control position")
        print("- Twist to control orientation")
        print("- Left button: toggle gripper")
        print("- Right button: None")
        print("- 'y' key: success")
        print("- 'n' key: failure")
        print("- 'space' key: reset origin")

    def reset_state(self):
        """Reset controller state"""
        self.interface._reset_internal_state()
        self._state = {
            "success": False,
            "failure": False,
            "movement_enabled": True,
            "controller_on": True,
        }
        self.interface._display_controls()
        
    def _limit_velocity(self, lin_vel, rot_vel, gripper_vel):
        """Limit linear and angular velocity magnitudes"""
        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)
        gripper_vel_norm = np.abs(gripper_vel)
        
        if lin_vel_norm > self.max_lin_vel:
            lin_vel = lin_vel * self.max_lin_vel / lin_vel_norm
        if rot_vel_norm > self.max_rot_vel:
            rot_vel = rot_vel * self.max_rot_vel / rot_vel_norm
        if gripper_vel_norm > self.max_gripper_vel:
            gripper_vel = gripper_vel * self.max_gripper_vel / gripper_vel_norm
            
        return lin_vel, rot_vel, gripper_vel

    def forward(self, obs_dict):
        """Calculate action with improved control response"""
        # Get current SpaceMouse state
        data = self.interface.get_controller_state()
        
        # Extract control inputs - dpos and drot are already remapped
        dpos = data["dpos"]
        drot = data["raw_drotation"]
        gripper_action = 1.0 if data["grasp"] else -1.0
        
        # Apply non-linear response curve for better fine control
        # This makes small movements more precise while allowing fast large movements
        lin_vel = self._apply_control_curve(dpos)
        rot_vel = self._apply_control_curve(drot, exponent=1.5)  # Less aggressive for rotation
        
        # Limit velocities
        lin_vel, rot_vel, grip_vel = self._limit_velocity(lin_vel, rot_vel, gripper_action)
        
        # Build complete action
        action = np.concatenate([lin_vel, rot_vel, [grip_vel]])
        action = np.clip(action, -1, 1)
        
        return action, {} # TODO info_dict 
    
    def _apply_control_curve(self, values, exponent=2.0):
        """Apply non-linear response curve for better control"""
        signs = np.sign(values)
        magnitudes = np.abs(values)
        return signs * (magnitudes ** exponent)

    def get_info(self):
        """Return controller info, compatible with other controller interfaces"""
        # Copy current state
        ret = self._state.copy()
        return ret
    
    def register_key(self, key):
        """Handle keyboard input, compatible with other controller interfaces"""
        if key == ord(" "):
            # Space key resets origin
            print("Resetting origin")
            self.interface._reset_internal_state()
        elif key == ord("y"):
            # 'y' key sets success
            print("Spacemouse teleop success")
            self._state["success"] = True
            self.interface._reset_internal_state()
        elif key == ord("n"): 
            # 'n' key sets failure
            print("Spacemouse teleop failure")
            self._state["failure"] = True
            self.interface._reset_internal_state()
    
    def close(self):
        """Close controller and release resources"""
        print("Closing SpaceMouse")
        # SpaceMouseInterface has no explicit close method, but can be added if needed
        pass


# if __name__ == "__main__":
#     interface = SpaceMouseInterface()
#     interface.start_control()

#     while True:
#         data = interface.get_controller_state()
#         positional_delta = data["dpos"]
#         print(data['raw_drotation'].round(3))
#         rotational_delta = R.from_matrix(data['rotation']).as_euler('xyz', degrees=False)
#         print(rotational_delta.round(3))
#         print(data)
#         print(data['dpos'].round(3))
#         time.sleep(0.5)