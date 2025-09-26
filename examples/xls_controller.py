import threading
import numpy as np
from pynput import keyboard     # Global keyboard listener
from typing import Dict
from dyros_robot_controller import KinematicParam, DriveType
from dyros_robot_controller.robot_data.mobile.base import MobileBase as RobotData
from dyros_robot_controller.robot_controller.mobile.base import MobileControllerBase as RobotController

class XLSController:
    def __init__(self, dt: float):
        # Simulation/control timestep
        self.dt = dt
        
        # Define mobile base kinematics (Mecanum drive) and constraints
        # All geometry is expressed in the base frame:
        #  - base2wheel_positions: 2D position of each wheel center wrt base origin
        #  - base2wheel_angles: wheel steering angles (fixed here)
        #  - roller_angles: Mecanum roller angles for each wheel
        #  - max_*: saturation limits (used by controller for velocity/acceleration limits)
        param = KinematicParam(
            type                 = DriveType.Mecanum,
            wheel_radius         = 0.120,
            base2wheel_positions = [np.array([ 0.2225,  0.2045]),   # Front Left
                                    np.array([ 0.2225, -0.2045]),   # Front Right
                                    np.array([-0.2225,  0.2045]),   # Rear Left
                                    np.array([-0.2225, -0.2045])],  # Rear Right
            base2wheel_angles    = [0,0,0,0],
            roller_angles        = [-np.pi/4,  # Front Left
                                     np.pi/4,  # Front Right
                                     np.pi/4,  # Rear Left
                                    -np.pi/4], # Rear Right 
            max_lin_speed        = 2,
            max_ang_speed        = 3,
            max_lin_acc          = 3,
            max_ang_acc          = 6,
        )

        # Initialize Dyros mobile robot model and controller
        self.robot_data = RobotData(param)
        self.robot_controller = RobotController(dt=self.dt,
                                                robot_data=self.robot_data)
        
        # Number of wheels (derived from param)
        self.wheel_num = self.robot_data.get_wheel_num()
        
        # Wheel states and command buffers
        self.wheel_pos           = np.zeros(self.wheel_num)  # measured wheel positions
        self.wheel_vel           = np.zeros(self.wheel_num)  # measured wheel velocities
        self.wheel_vel_desired   = np.zeros(self.wheel_num)  # desired wheel velocities (control input)
        
        # Desired base twist in base frame: [vx, vy, w]
        self.base_vel_desired    = np.zeros(3)

        # --- Control mode bookkeeping ---
        self.control_mode = "Stop"         # Default: stop (zero command)
        self.is_control_mode_changed = True
        self.sim_time = 0.0
        self.control_start_time = 0.0
        
        # Start a global keyboard listener for non-blocking input
        # We track currently pressed keys to allow multi-key combos (e.g., Up+Right)
        self._pressed_keys = set()
        self._keys_lock = threading.Lock() 
        self._listener = keyboard.Listener(on_press=self._on_key_press, on_release=self._on_key_release)
        self._listener.daemon = True
        self._listener.start()


    def update_model(self, current_time: float, qpos_dict: Dict, qvel_dict: Dict):
        # Update simulation time from MuJoCo
        self.sim_time = current_time

        # Map sim joint dictionaries -> ordered wheel arrays
        # NOTE: The order must match the order assumed by KinematicParam above.
        self.wheel_pos[:] = np.array([qpos_dict["front_left_wheel"],   # Front Left
                                      qpos_dict["front_right_wheel"],  # Front Right
                                      qpos_dict["rear_left_wheel"],    # Rear Left
                                      qpos_dict["rear_right_wheel"]])  # Rear Right
        self.wheel_vel[:] = np.array([qvel_dict["front_left_wheel"],
                                      qvel_dict["front_right_wheel"],
                                      qvel_dict["rear_left_wheel"],
                                      qvel_dict["rear_right_wheel"]])
        
        # Push latest wheel states into the model
        self.robot_data.update_state(self.wheel_pos, self.wheel_vel)

    def compute(self):
        # One-time initialization per mode entry
        if self.is_control_mode_changed:
            self.is_control_mode_changed = False
            self.control_start_time = self.sim_time
            
        # --- Mode: Stop (zero command) ---
        if self.control_mode == "Stop":
            self.wheel_vel_desired[:] = 0.0
            
        # --- Mode: Base Velocity Tracking ---
        # Convert current key set -> desired base twist [vx, vy, w]
        # Then compute wheel velocities via the controller's mapping
        elif self.control_mode == "Base Velocity Tracking":
            self.base_vel_desired = self._keys_to_xdot()
            self.wheel_vel_desired = self.robot_controller.velocity_command(self.base_vel_desired)
      
        # Format controller outputs for MuJoCo (actuator name -> value)
        return {
            "front_left_wheel":  float(self.wheel_vel_desired[0]),
            "front_right_wheel": float(self.wheel_vel_desired[1]),
            "rear_left_wheel":  float(self.wheel_vel_desired[2]),
            "rear_right_wheel": float(self.wheel_vel_desired[3]),
        }
    
    def set_mode(self, control_mode: str):
        # Switch control mode and trigger per-mode re-initialization in compute()
        self.is_control_mode_changed = True
        self.control_mode = control_mode
        print(f"Control Mode Changed: {self.control_mode}")
    
        
    def _on_key_press(self, key):
        # Track “held” keys to support continuous velocity while pressed
        with self._keys_lock:
            self._pressed_keys.add(key)

        # Numeric hotkeys for mode switching
        try:
            if key.char == '1':
                self.set_mode("Stop")
            elif key.char == '2':
                self.set_mode("Base Velocity Tracking")
            elif key.char == '3':
                self.set_mode("Gravity Compensation")  # (kept for parity; not used in compute)
        except AttributeError:
            # Special keys (arrows, etc.) don’t have .char
            pass

    def _on_key_release(self, key):
        # Remove key from the "held" set → that axis goes to 0 next cycle
        with self._keys_lock:
            if key in self._pressed_keys:
                self._pressed_keys.remove(key)
                
    def _keys_to_xdot(self) -> np.ndarray:
        """Map currently held keys to a base twist [vx, vy, w] in the base frame.
        - Arrow keys control linear vx, vy
        - 'b' increases yaw rate (+w), 'v' decreases yaw rate (-w)
        Releasing a key removes it from the set, so the corresponding component becomes 0.
        """
        vx = vy = w = 0.0
        with self._keys_lock:
            held = set(self._pressed_keys)

        # Linear velocity with arrow keys
        if keyboard.Key.up in held:    vx += 1.0   # forward
        if keyboard.Key.down in held:  vx -= 1.0   # backward
        if keyboard.Key.right in held: vy += 1.0   # left-to-right strafe
        if keyboard.Key.left in held:  vy -= 1.0   # right-to-left strafe

        # Yaw rate with letters ('b' = +w, 'v' = -w)
        if 'b' in [getattr(k, 'char', None) for k in held]:
            w += 1.0
        if 'v' in [getattr(k, 'char', None) for k in held]:
            w -= 1.0

        # Return desired base twist; controller will map to wheel velocities
        return np.array([vx, vy, w], dtype=float)
