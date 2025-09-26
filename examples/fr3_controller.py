import os
import numpy as np
from pynput import keyboard     # Global keyboard listener
from typing import Dict
from dyros_robot_controller.robot_data.manipulator.base import ManipulatorBase as RobotData
from dyros_robot_controller.robot_controller.manipulator.base import ManipulatorControllerBase as RobotController

class FR3Controller:
    def __init__(self, dt: float):
        # Control timestep (from simulation)
        self.dt = dt
        
        # Paths to URDF / SRDF files for the robot model
        urdf_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robots", "franka_fr3", "fr3.urdf")
        srdf_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robots", "franka_fr3", "fr3.srdf")

        # Initialize robot model and controller (Dyros Robot Controller)
        self.robot_data = RobotData(urdf_path=urdf_file_path,
                                    srdf_path=srdf_file_path,
                                    packages_path="None")
        self.robot_controller = RobotController(dt=self.dt,
                                                robot_data=self.robot_data)
        
        # Number of degrees of freedom
        self.dof = self.robot_data.get_dof()
        
        # --- Joint space state variables ---
        self.q            = np.zeros(self.dof)  # Current joint positions
        self.qdot         = np.zeros(self.dof)  # Current joint velocities
        self.q_desired    = np.zeros(self.dof)  # Desired joint positions
        self.qdot_desired = np.zeros(self.dof)  # Desired joint velocities
        self.q_init       = np.zeros(self.dof)  # Initial joint positions at mode start
        self.qdot_init    = np.zeros(self.dof)  # Initial joint velocities at mode start
        self.tau_desired  = np.zeros(self.dof)  # Desired joint torques (control input)
        
        # --- Task space (End-Effector) state variables ---
        self.x            = np.eye(4)      # Current end-effector pose (homogeneous transform)
        self.xdot         = np.zeros(6)    # Current end-effector velocity (linear + angular)
        self.x_desired    = np.eye(4)      # Desired end-effector pose
        self.xdot_desired = np.zeros(6)    # Desired end-effector velocity
        self.x_init       = np.eye(4)      # Initial end-effector pose at mode start
        self.xdot_init    = np.zeros(6)    # Initial end-effector velocity at mode start
        self.ee_link_name = "fr3_link8"    # End-effector link name from FR3 URDF
        
        # --- Control data ---
        self.control_mode = "Home"         # Default control mode
        self.is_control_mode_changed = True
        self.sim_time = 0.0
        self.control_start_time = 0.0
        
        # Start a global keyboard listener (runs in a background thread)
        self._listener = keyboard.Listener(on_press=self._on_key_press)
        self._listener.daemon = True
        self._listener.start()


    def update_model(self, current_time: float, qpos_dict: Dict, qvel_dict: Dict):
        # Update simulation time
        self.sim_time = current_time

        # Read joint positions/velocities from simulation dictionaries
        for i in range(self.dof):
            self.q[i]    = qpos_dict["fr3_joint"+str(i+1)] # Joint names in fr3.xml: fr3_joint1..7
            self.qdot[i] = qvel_dict["fr3_joint"+str(i+1)]
        
        # Update kinematics/dynamics in RobotData
        self.robot_data.update_state(self.q, self.qdot)
        self.x    = self.robot_data.get_pose(self.ee_link_name)
        self.xdot = self.robot_data.get_velocity(self.ee_link_name)

    def compute(self):
        # When control mode changes → reset reference states
        if self.is_control_mode_changed:
            self.is_control_mode_changed = False
            self.control_start_time = self.sim_time
            self.q_init    = self.q.copy()
            self.qdot_init = self.qdot.copy()
            self.x_init    = self.x.copy()
            self.xdot_init = self.xdot.copy()
            self.q_desired    = self.q_init.copy()
            self.qdot_desired = np.zeros_like(self.qdot)
            self.x_desired    = self.x_init.copy()
            self.xdot_desired = np.zeros_like(self.xdot)
            
        # --- Control mode: Home (move to a predefined joint position) ---
        if self.control_mode == "Home":
            q_home = np.array([0.0, 0.0, 0.0, -np.pi/2., 0.0, np.pi/2., np.pi / 4.])
            self.q_desired = self.robot_controller.move_joint_position_cubic(q_target=q_home,
                                                                             qdot_target=np.zeros(self.dof),
                                                                             q_init=self.q_init,
                                                                             qdot_init=self.qdot_init,
                                                                             init_time=self.control_start_time,
                                                                             current_time=self.sim_time,
                                                                             duration=3.0)
            self.qdot_desired = self.robot_controller.move_joint_velocity_cubic(q_target=q_home,
                                                                                qdot_target=np.zeros(self.dof),
                                                                                q_init=self.q_init,
                                                                                qdot_init=self.qdot_init,
                                                                                init_time=self.control_start_time,
                                                                                current_time=self.sim_time,
                                                                                duration=3.0)
            self.tau_desired = self.robot_controller.move_joint_torque_step(q_target=self.q_desired,
                                                                            qdot_target=self.qdot_desired)
            
            # Alternative: directly synthesize torque over time with a single API (kept commented per original code)
            # self.tau_desired = self.robot_controller.move_joint_torque_cubic(...)
            
        # --- Control mode: QPIK (Inverse Kinematics using QP in task space) ---
        elif self.control_mode == "QPIK":
            target_x = self.x_init.copy()
            target_x[:3, 3] = target_x[:3, 3] + np.array([0., 0.1, 0.1])  # Move EE by +10 cm in Y and +10 cm in Z from current pose.
            self.qdot_desired = self.robot_controller.QPIK_cubic(x_target=target_x,
                                                                 xdot_target=np.zeros(6),
                                                                 x_init=self.x_init,
                                                                 xdot_init=self.xdot_init,
                                                                 init_time=self.control_start_time,
                                                                 current_time=self.sim_time,
                                                                 duration=2.0,
                                                                 link_name=self.ee_link_name)
            self.q_desired = self.q.copy() + self.qdot_desired.copy() * self.dt
            
            # Map desired (q, qdot) to torque command (gravity + PD)
            self.tau_desired = self.robot_controller.move_joint_torque_step(q_target=self.q_desired,
                                                                            qdot_target=self.qdot_desired)
        
        # --- Control mode: Gravity Compensation ---
        elif self.control_mode == "Gravity Compensation":
            self.tau_desired = self.robot_data.get_gravity() # Pure gravity compensation torque from model (no tracking).
            
        # Return control input dictionary (name → torque)
        ctrl_dict = {}
        for i in range(self.dof):
            ctrl_dict["fr3_joint"+str(i+1)] = self.tau_desired[i] # Actuator names in fr3.xml: fr3_joint1..7
        
        return ctrl_dict
    
    def set_mode(self, control_mode: str):
        # Change mode and flag reset
        self.is_control_mode_changed = True
        self.control_mode = control_mode
        print(f"Control Mode Changed: {self.control_mode}")
    
    def _on_key_press(self, key):
        # Keyboard shortcuts for control mode switching
        try:
            if key.char == '1':
                self.set_mode("Home")
            elif key.char == '2':
                self.set_mode("QPIK")
            elif key.char == '3':
                self.set_mode("Gravity Compensation")
        except AttributeError:
            # Non-character keys ignored
            pass
