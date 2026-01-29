'''
Robotiq Gripper Interface.

Author: Jinzhou Li

References:
  [1] https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_Instruction_Manual_e-Series_PDF_20190206.pdf
  [2] https://github.com/castetsb/pyRobotiqGripper
'''

import time
import serial
import threading
import numpy as np
import minimalmodbus as mm


class Robotiq2FGripper():
    '''
    Robotiq Gripper (2F-85, 2F-140) Interface using Modbus RTU.
    '''
    def __init__(
        self,
        port: str,
        slave_address: int = 9,
        timeout: float = 10.0,
    ):
        '''
        Initialization.

        Parameters:
        - port: str, required, the port of the Robotiq 2F-(85/140) Gripper;
        - slave_address: int, optional, default: 9, Modbus slave address;
        - timeout: float, optional, default: 10.0, timeout for operations in seconds.
        '''
        self.port = port
        self.slave_address = slave_address
        self.timeout = timeout
        self.waiting_gap = 0.01
        self.last_position = 255
        self.last_speed = 100
        self.last_force = 77
        self.last_timestamp = int(time.time() * 1000)

        # Create serial connection
        ser = serial.Serial(
            port=self.port,
            baudrate=115200,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=0.2
        )

        # Create minimalmodbus instrument
        self.instrument = mm.Instrument(
            ser,
            self.slave_address,
            mm.MODE_RTU,
            close_port_after_each_call=False,
            debug=False
        )

        self.lock = threading.Lock()
        self.paramDic = {}

        self.activate()
    
    def _read_all(self):
        '''Read all gripper registers and update paramDic.'''
        with self.lock:
            registers = self.instrument.read_registers(2000, 3)

        # Register 2000: gripper status
        gripper_status = (registers[0] >> 8) & 0xFF
        self.paramDic["gOBJ"] = (gripper_status >> 6) & 0b11  # Object detection
        self.paramDic["gSTA"] = (gripper_status >> 4) & 0b11  # Gripper status
        self.paramDic["gGTO"] = (gripper_status >> 3) & 0b1   # Go-to status
        self.paramDic["gACT"] = gripper_status & 0b1          # Activation status

        # Register 2001: fault status and position echo
        fault_status = (registers[1] >> 8) & 0xFF
        self.paramDic["kFLT"] = (fault_status >> 4) & 0xF
        self.paramDic["gFLT"] = fault_status & 0xF
        self.paramDic["gPR"] = registers[1] & 0xFF  # Position request echo

        # Register 2002: position and current
        self.paramDic["gPO"] = (registers[2] >> 8) & 0xFF  # Actual position
        self.paramDic["gCU"] = registers[2] & 0xFF         # Current

    def activate(self):
        '''
        Activate the gripper.
        Warning: Gripper will open and close during activation.
        '''
        # Reset first
        with self.lock:
            self.instrument.write_registers(1000, [0, 0, 0])
        time.sleep(0.1)

        # Send activation command (rACT=1)
        with self.lock:
            self.instrument.write_registers(1000, [0b0000000100000000, 0, 0])

        # Wait for activation to complete
        start_time = time.time()
        while time.time() - start_time < self.timeout:
            self._read_all()
            if self.paramDic["gSTA"] == 3:
                return
            time.sleep(self.waiting_gap)

        raise TimeoutError("Gripper activation timed out.")

    def open_gripper(self, speed=255, force=255):
        '''
        Open the gripper.

        Parameters:
        - speed: int, optional, default: 255, opening speed (0-255);
        - force: int, optional, default: 255, force (0-255).
        '''
        self._go_to(0, speed, force)

    def close_gripper(self, speed=255, force=255):
        '''
        Close the gripper.

        Parameters:
        - speed: int, optional, default: 255, closing speed (0-255);
        - force: int, optional, default: 255, force (0-255).
        '''
        self._go_to(255, speed, force)

    def _go_to(self, position, speed=255, force=255):
        '''
        Move gripper to position and wait for completion.

        Parameters:
        - position: int, target position (0=open, 255=closed);
        - speed: int, movement speed (0-255);
        - force: int, gripping force (0-255).

        Returns:
        - (position, object_detected): tuple of final position and bool.
        '''
        position = int(min(255, max(0, position)))
        speed = int(min(255, max(0, speed)))
        force = int(min(255, max(0, force)))

        # rGTO=1 (go to), rACT=1 (activated)
        with self.lock:
            self.instrument.write_registers(1000, [
                0b0000100100000000,
                position,
                speed * 256 + force
            ])

        self.last_position = position
        self.last_speed = speed
        self.last_force = force
        self.last_timestamp = int(time.time() * 1000)

        # Wait for motion to complete
        start_time = time.time()
        while time.time() - start_time < self.timeout:
            self._read_all()
            gOBJ = self.paramDic["gOBJ"]

            if gOBJ == 1 or gOBJ == 2:
                # Object detected
                return self.paramDic["gPO"], True
            elif gOBJ == 3:
                # At requested position, no object
                return self.paramDic["gPO"], False

            time.sleep(self.waiting_gap)

        raise TimeoutError("Gripper motion timed out.")

    def action(self, position, speed=100, force=77, **kwargs):
        '''
        Send control command to the gripper (non-blocking).

        Parameters:
        - position: int, required, the position of the gripper (0-255);
        - speed: int, optional, default: 100, speed (0-255);
        - force: int, optional, default: 77, force (0-255).
        '''
        position = int(min(255, max(0, position)))
        speed = int(min(255, max(0, speed)))
        force = int(min(255, max(0, force)))

        with self.lock:
            self.instrument.write_registers(1000, [
                0b0000100100000000,
                position,
                speed * 256 + force
            ])

        self.last_position = position
        self.last_speed = speed
        self.last_force = force
        self.last_timestamp = int(time.time() * 1000)

    def get_info(self):
        '''
        Get current gripper information.

        Returns:
        - numpy array: [position, current, status, last_position, last_force, last_speed, last_timestamp]
          - position: 0-255
          - current: 0-255 (multiply by 10 for mA)
          - status: 0=open complete, 1=close complete, 2=opening, 3=closing, -1=error
        '''
        self._read_all()

        position = self.paramDic["gPO"]
        current = self.paramDic["gCU"]
        gOBJ = self.paramDic["gOBJ"]
        gFLT = self.paramDic["gFLT"]

        # Check for faults (allow 0=no fault and 9=no communication)
        if gFLT != 0 and gFLT != 9:
            status = -1
        else:
            # gOBJ: 0=moving, 1=object opening, 2=object closing, 3=at position
            if gOBJ == 0:
                # Moving - check direction based on last command
                status = 3 if self.last_position > 127 else 2
            elif gOBJ == 1:
                status = 0  # Stopped while opening
            elif gOBJ == 2:
                status = 1  # Stopped while closing (object detected)
            else:  # gOBJ == 3
                # At position - determine if open or closed
                status = 1 if position > 127 else 0

        return np.array([
            position, current, status,
            self.last_position, self.last_force, self.last_speed, self.last_timestamp
        ]).astype(np.int64)

    def grasp(self, speed=50, force=30):
        '''
        Close the gripper with specified speed and force, wait for completion.

        Parameters:
        - speed: int, optional, default: 50, closing speed (0-255);
        - force: int, optional, default: 30, grasping force (0-255).

        Returns:
        - (position, object_detected): tuple of final position and bool.
        '''
        return self._go_to(255, speed, force)
    
if __name__ == "__main__":

    gripper = Robotiq2FGripper(port='/dev/ttyUSB0')
    gripper.grasp(speed=50, force=20)
    print('Grasped.')
    weidth = gripper.get_info()[0]
    print('Weidth after grasping: {}'.format(weidth))
