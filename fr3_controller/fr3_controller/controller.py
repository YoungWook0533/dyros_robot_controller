import numpy as np
from .utils import cubic_spline, ControlledThread
from mujoco_ros_sim import ControllerInterface
from .robot_data import FR3RobotData
from rclpy.node import Node
from fr3_controller_wrapper_cpp import Controller as Controllercpp
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

class FR3Controller(ControllerInterface):
    """
    Controller implementation for the FR3 robot.
    
    This controller uses FR3RobotData to update the robot's state and subscribes to key inputs
    to change the control mode. It computes desired joint positions using cubic spline interpolation
    in an asynchronous thread. The controller provides control commands that are applied to the robot's actuators.
    
    Attributes:
        robot_data (FR3RobotData): Instance for managing the FR3 robot's data.
        is_mode_changed (bool): Flag indicating if the control mode has changed.
        key_subscriber: ROS2 subscription for key input messages.
        q_desired (np.ndarray): Desired joint positions computed by the controller.
        mode (str): Current control mode (e.g., 'home').
        current_time (float): Latest simulation time (updated in updateState).
        q_init (np.ndarray): Initial joint positions recorded at the start of a control sequence.
        control_start_time (float): Simulation time when the current control sequence started.
    """
    
    def __init__(self, node: Node, dt: float, mj_joint_names: list):
        """
        Initializes the FR3Controller.
        
        Parameters:
            node (Node): A ROS2 node instance used for logging, subscriptions, and communication.
            dt (float): Simulation time step in seconds.
            mj_joint_names (list): List of joint names from the MuJoCo simulation.
            
        Returns:
            None
        """
        super().__init__(node, dt, mj_joint_names)
        
        self.key_subscriber = self.node.create_subscription(Int32, 'fr3_controller/mode_input', self.keyCallback, 10)
        self.robot_data = FR3RobotData(self.node, mj_joint_names)
        
        self.target_pose = self.node.create_subscription(PoseStamped, 'fr3_controller/target_pose', self.targetPoseCallback, 10)
            
    def starting(self) -> None:
        """
        Called once when the controller starts.
        Captures the initial joint positions from the robot data and records the control start time.
    
        Returns:
            None
        """
        self.is_mode_changed = False
        self.mode = 'home'
        self.control_start_time = self.current_time
        
        self.q_init = self.q
        self.q_desired = self.q_init
        self.target_pose = self.robot_data.getPose()
        self.torque_desired = self.tau_ext
        
    def updateState(self, pos: np.ndarray, vel: np.ndarray, tau_ext: np.ndarray, current_time: float) -> None:
        """
        Updates the controller's state with the latest simulation data.
        This method updates the internal simulation time, refreshes the robot state via FR3RobotData,
        and logs the current joint positions for debugging.
        
        Parameters:
            pos (np.ndarray): Current joint positions from the MuJoCo simulation.
            vel (np.ndarray): Current joint velocities from the MuJoCo simulation.
            tau_ext (np.ndarray): Current external joint torques from the MuJoCo simulation.
            current_time (float): The current simulation time.
            
        Returns:
            None
        """
        # Update the current simulation time.
        self.current_time = current_time
        
        # Update the robot's state using the FR3RobotData interface.
        self.robot_data.updateState(pos, vel, tau_ext)
        
        self.q = self.robot_data.q
        self.qdot = self.robot_data.qdot
        self.tau_ext = self.robot_data.tau_ext
        
        # self.node.get_logger().info(f"tau_ext: {self.tau_ext}")
        
    def compute(self) -> None:
        """
        Computes the control commands for the FR3 robot.
        Launches an asynchronous thread to compute the desired joint positions and waits for the
        computation to complete within the allocated simulation time step. If the computation exceeds
        the allowed time, it attempts to forcibly terminate the thread.
        
        Returns:
            None
        """
        # Start an asynchronous thread to execute the control calculation.
        t = ControlledThread(target=self.asyncCalculationProc)
        t.start()

        # Wait for the thread to finish, up to a maximum of dt seconds.
        t.join(self.dt)

        # If the thread is still running after dt seconds, attempt to kill it.
        if t.is_alive():
            self.node.get_logger().info("[FR3controller] 제한 시간 초과, 스레드 종료 시도")
            t.kill()
            
    def getCtrlInput(self) -> tuple[np.ndarray, list]:
        """
        Retrieves the computed control commands to be applied to the robot.
        
        Returns:
            tuple:
                - np.ndarray: Array containing the desired joint positions (control commands).
                - list: List of robot joint names corresponding to the control commands.
        """
        return self.torque_desired, self.robot_data.rb_joint_names
    
    # =============================================================================================
    def keyCallback(self, msg: Int32):
        """
        Callback function for key input messages.
        If a message with data value 1 is received, the control mode is set to 'home'.
        
        Parameters:
            msg (Int32): The received key input message containing an integer.
            
        Returns:
            None
        """
        self.node.get_logger().info(f"[FR3Controller] Key input received: {msg.data}")
        if msg.data == 1:
            self.setMode('init')
        elif msg.data == 2:
            self.setMode('home')
                    
    def setMode(self, mode: str):
        """
        Sets the current control mode and flags that a mode change has occurred.
        Also logs the mode change.
        
        Parameters:
            mode (str): The new control mode (e.g., 'home').
            
        Returns:
            None
        """
        self.is_mode_changed = True
        self.node.get_logger().info(f"[FR3Controller] Mode changed: {mode}")
        self.mode = mode
        
    def asyncCalculationProc(self):
        """
        Performs asynchronous control computations in a separate thread.
        """
        if self.is_mode_changed:
            self.q_init = self.robot_data.q
            self.control_start_time = self.current_time
            self.is_mode_changed = False
        
        # Compute desired joint positions based on the current control mode.
        if self.mode == 'init':
            target_q = np.array([0, 0, 0, 0, 0, 0, np.pi/4])
            self.q_desired = cubic_spline(
                self.current_time,              # Current time
                self.control_start_time,        # Start time of control
                self.control_start_time + 2.0,  # End time of interpolation (2 seconds later)
                self.q_init,                    # Initial joint positions
                target_q,                       # Target joint positions
                np.zeros(7),                    # Initial velocity (assumed zero)
                np.zeros(7)                     # Final velocity (assumed zero)
            )
            self.torque_desired = self.PDControl(self.q_desired, np.zeros(7), 1000, 100)
        elif self.mode == 'home':
            target_q = np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
            self.q_desired = cubic_spline(
                self.current_time,              # Current time
                self.control_start_time,        # Start time of control
                self.control_start_time + 2.0,  # End time of interpolation (2 seconds later)
                self.q_init,                    # Initial joint positions
                target_q,                       # Target joint positions
                np.zeros(7),                    # Initial velocity (assumed zero)
                np.zeros(7)                     # Final velocity (assumed zero)
            )
            self.torque_desired = self.PDControl(self.q_desired, np.zeros(7), 1000, 100)
        else:
            self.torque_desired = np.zeros(7)

    def PDControl(self, q_desired:np.ndarray, qdot_desired:np.ndarray, kp:float, kd:float) -> np.ndarray:
        """
        PD control law for joint position and velocity tracking.
        
        Parameters:
            q_desired (np.ndarray): Desired joint positions.
            qdot_desired (np.ndarray): Desired joint velocities.
            kp (float): Proportional gain for position control.
            kd (float): Derivative gain for velocity control.
            
        Returns:
            np.ndarray: Torque commands computed by the PD controller.
        """
        q_error = q_desired - self.q
        qdot_error = qdot_desired - self.qdot
        tau = self.robot_data.getMassMatrix() @ (kp * q_error + kd * qdot_error) + self.robot_data.getCoriolis()
        return tau
    
    def targetPoseCallback(self, msg: PoseStamped):
        quat = [msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w]

        T = np.eye(4)
        T[:3, :3] = R.from_quat(quat).as_matrix()
        T[:3, 3] = [msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z]
        
        self.target_pose = T