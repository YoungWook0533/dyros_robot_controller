import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import threading
from scipy.spatial.transform import Rotation as R
import threading
import time

from dyros_robot_controller.utils import cubic_spline, cubic_dot_spline, rotation_cubic, rotation_cubic_dot, get_phi

from mujoco_ros_sim import ControllerInterface
from .robot_data import FR3RobotData

from dyros_robot_controller_wrapper_cpp import FR3Controller as Controllercpp

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from srmt2.planning_scene import PlanningScene
from srmt2.planner.rrt_connect import SuhanRRTConnect
from srmt2.kinematics import TRACIK

"""
FR3 MuJoCo Joint/Sensor Imformation
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  0 | fr3_joint1           | _Hinge |  1 |  1 |     0 |    0
  1 | fr3_joint2           | _Hinge |  1 |  1 |     1 |    1
  2 | fr3_joint3           | _Hinge |  1 |  1 |     2 |    2
  3 | fr3_joint4           | _Hinge |  1 |  1 |     3 |    3
  4 | fr3_joint5           | _Hinge |  1 |  1 |     4 |    4
  5 | fr3_joint6           | _Hinge |  1 |  1 |     5 |    5
  6 | fr3_joint7           | _Hinge |  1 |  1 |     6 |    6

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | fr3_joint1           | _Joint  | fr3_joint1
  1 | fr3_joint2           | _Joint  | fr3_joint2
  2 | fr3_joint3           | _Joint  | fr3_joint3
  3 | fr3_joint4           | _Joint  | fr3_joint4
  4 | fr3_joint5           | _Joint  | fr3_joint5
  5 | fr3_joint6           | _Joint  | fr3_joint6
  6 | fr3_joint7           | _Joint  | fr3_joint7

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
"""
class FR3Controller(ControllerInterface):
    
    def __init__(self, node: Node, dt: float, mj_joint_dict: list):
        super().__init__(node, dt, mj_joint_dict)
        
        self.robot_data = FR3RobotData(self.node, mj_joint_dict["joint_names"])
        
        self.key_sub = self.node.create_subscription(Int32, 'fr3_controller/mode_input', self.keyCallback, 10)
        self.target_pose_sub = self.node.create_subscription(Pose, 'fr3_controller/target_pose', self.targetPoseCallback, 10)
        
        self.EEpose_pub = self.node.create_publisher(Pose, 'fr3_controller/ee_pose', 10)
        
        urdf_path = os.path.join(
            get_package_share_directory('dyros_robot_controller'),
            'robot',
            'fr3.urdf'
        )
        srdf_path = os.path.join(
            get_package_share_directory('fr3_moveit_config'),
            'config',
            'fr3.srdf'
        )
        self.pc = PlanningScene(arm_names=["fr3_arm"], 
                                arm_dofs=[self.robot_data.dof], 
                                base_link="fr3_link0", 
                                node_name="fr3_PlanningScene",
                                topic_name="/fr3_planning_scene",
                                urdf_file_path=urdf_path,
                                srdf_file_path=srdf_path,)
        
        self.tracIK = TRACIK(base_link="fr3_link0", 
                             tip_link="fr3_link8", 
                             urdf_file_path=urdf_path)
        
        self.joint_upper_limit = self.tracIK.get_upper_bound()
        self.joint_lower_limit = self.tracIK.get_lower_bound()
        
        self.rrt_planner = SuhanRRTConnect(state_dim=self.robot_data.dof, 
                                           lb=self.joint_lower_limit, 
                                           ub=self.joint_upper_limit, 
                                           validity_fn=self.pc.is_valid)
        
        self.compute_slow_thread = threading.Thread(target=self.computeSlow)
       
    def starting(self) -> None:
        self.is_mode_changed = False
        self.mode = 'home'
        self.control_start_time = self.current_time
        
        self.q_init = self.q
        self.q_desired = self.q_init
        self.target_pose = self.robot_data.getPose()
        self.torque_desired = self.tau_ext
        
        self.x_init = self.x
        self.target_pose = self.x_init
        self.x_desired = self.x_init
        self.xdot_init = self.xdot
        self.xdot_desired = np.zeros(6)
        
        self.is_target_pose_changed = False
        self.is_rrt_path_found = False
        
        self.node.create_timer(0.01, self.pub_EEpose_callback)
        
        self.compute_slow_thread.start()
        
    def updateState(self, 
                    pos_dict: dict, 
                    vel_dict: dict, 
                    tau_ext_dict: dict,
                    sensor_dict: dict, 
                    current_time: float) -> None:
        
        self.current_time = current_time
        self.robot_data.updateState(pos_dict, vel_dict, tau_ext_dict, sensor_dict)
        
        self.q = self.robot_data.q
        self.qdot = self.robot_data.qdot
        self.tau_ext = self.robot_data.tau_ext
        self.x = self.robot_data.getPose()
        self.xdot = self.robot_data.getVelocity()
        
        self.pc.display(self.q)
        
        
    def compute(self) -> None:
        if self.is_mode_changed:
            self.q_init = self.robot_data.q
            self.qdot_init = self.robot_data.qdot
            self.q_desired = self.q_init
            self.qdot_desired = np.zeros(7)
            self.x_init = self.x
            self.x_desired = self.x_init
            self.xdot_init = self.xdot
            self.xdot_desired = np.zeros(6)
            self.target_pose = self.x_init
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
        elif self.mode == 'RRTmove':
            self.torque_desired = self.PDControl(self.q_desired, self.qdot_desired, 1000, 100)
        elif self.mode == 'CLIK':
            self.q_desired = self.CLIK(self.target_pose, duration=2.0)
            self.torque_desired = self.PDControl(self.q_desired, self.qdot_desired, 1000, 100)
        else:
            self.torque_desired = np.zeros(7)
            
    def computeSlow(self) -> None:
        while rclpy.ok():
            if self.is_mode_changed:
                self.is_rrt_path_found = False
                self.is_target_pose_changed = False
            
            # Compute desired joint positions based on the current control mode.
            if self.mode == 'RRTmove':
                if self.target_pose is not None and self.is_target_pose_changed:
                    self.is_target_pose_changed = False
                    for _ in range(10):
                        self.node.get_logger().info("Attempting to solve IK...")
                        is_ik_found, q_goal = self.tracIK.solve(pos=self.target_pose[0:3,3],
                                                    quat=R.from_matrix(self.target_pose[0:3,0:3]).as_quat(), 
                                                    q_init=self.q)
                        if is_ik_found:
                            self.node.get_logger().info(f"IK solution found:\n{q_goal}")
                            if self.pc.is_valid(q_goal):
                                self.node.get_logger().info("IK solution is valid")
                                break
                            else:
                                self.node.get_logger().info('fr3 is in collision')
                                is_ik_found = False
                        else:
                            self.node.get_logger().info('fr3 IK failed')
                            
                    if is_ik_found:
                        self.rrt_planner.max_distance = 0.1
                        self.rrt_planner.set_start(self.q)
                        self.rrt_planner.set_goal(q_goal)
                        for _ in range(10):
                            self.node.get_logger().info("Attempting to find RRT path...")
                            self.is_rrt_path_found, path = self.rrt_planner.solve()
                            if self.is_rrt_path_found:
                                self.node.get_logger().info("Path found")
                                self.traj_q, self.traj_qdot, self.traj_qddot, self.traj_time = self.pc.time_parameterize(path, 
                                                                                                    max_velocity_scaling_factor=1.0, 
                                                                                                    max_acceleration_scaling_factor=1.0)
                                self.traj_time = self.current_time + self.traj_time - self.traj_time[0]
                                break
                if self.is_rrt_path_found:
                    for i in range(len(self.traj_time)-1):
                        if self.current_time >= self.traj_time[i] and self.current_time < self.traj_time[i+1]:                            
                            self.q_desired = cubic_spline(time=self.current_time,
                                                            time_0=self.traj_time[i],
                                                            time_f=self.traj_time[i+1],
                                                            x_0=self.traj_q[i],
                                                            x_f=self.traj_q[i+1], 
                                                            x_dot_0=self.traj_qdot[i], 
                                                            x_dot_f=self.traj_qdot[i+1])
                            self.qdot_desired = cubic_dot_spline(time=self.current_time,
                                                                    time_0=self.traj_time[i],
                                                                    time_f=self.traj_time[i+1],
                                                                    x_0=self.traj_q[i],
                                                                    x_f=self.traj_q[i+1], 
                                                                    x_dot_0=self.traj_qdot[i], 
                                                                    x_dot_f=self.traj_qdot[i+1])
                            break
            time.sleep(0.001)
    
    def getCtrlInput(self) -> dict:
        ctrl_dict = {}
        for i, joint_name in enumerate(self.robot_data.rd_joint_names):
            ctrl_dict[joint_name] = self.torque_desired[i]
        return ctrl_dict
    
    # =============================================================================================
    def keyCallback(self, msg: Int32):
        self.node.get_logger().info(f"[FR3Controller] Key input received: {msg.data}")
        if msg.data == 1:
            self.setMode('init')
        elif msg.data == 2:
            self.setMode('home')
        elif msg.data == 3:
            self.setMode('RRTmove')
        elif msg.data == 4:
            self.setMode('CLIK')
                    
    def setMode(self, mode: str):
        self.is_mode_changed = True
        self.node.get_logger().info(f"[FR3Controller] Mode changed: {mode}")
        self.mode = mode
        
    def CLIK(self, 
             x_target: np.ndarray, 
             duration: float = 2.0,
             kp: np.ndarray = np.array([10, 10, 10, 10, 10, 10]),
             kd: np.ndarray = np.array([5, 5, 5, 5, 5, 5])) -> np.ndarray:
        
        control_start_time = self.control_start_time
        x_init = self.x_init
        xdot_init = self.xdot_init
        if self.is_target_pose_changed:
            self.is_target_pose_changed = False
            control_start_time = self.current_time
            x_init = self.x
            xdot_init = self.xdot
        x_desired = np.eye(4)
        xdot_desired = np.zeros(6)
        x_desired[0:3, 3] = cubic_spline(time=self.current_time,
                                          time_0=control_start_time,
                                          time_f=control_start_time + duration,
                                          x_0=x_init[0:3, 3],
                                          x_f=x_target[0:3, 3],
                                          x_dot_0=xdot_init[0:3],
                                          x_dot_f=np.zeros(3))
        x_desired[0:3, 0:3] = rotation_cubic(time=self.current_time,
                                             time_0=control_start_time,
                                             time_f=control_start_time + duration,
                                             R_0=x_init[0:3, 0:3],
                                             R_f=x_target[0:3, 0:3])
        xdot_desired[0:3] = cubic_dot_spline(time=self.current_time,
                                             time_0=control_start_time,
                                             time_f=control_start_time + duration,
                                             x_0=x_init[0:3, 3],
                                             x_f=x_target[0:3, 3],
                                             x_dot_0=xdot_init[0:3],
                                             x_dot_f=np.zeros(3))
        xdot_desired[3:6] = rotation_cubic_dot(time=self.current_time,
                                               time_0=control_start_time,
                                               time_f=control_start_time + duration,
                                               R_0=x_init[0:3, 0:3],
                                               R_f=x_target[0:3, 0:3])
        self.x_desired = x_desired
        self.xdot_desired = xdot_desired
        
        x_error = np.zeros(6)
        x_error[0:3] = self.x_desired[0:3, 3] - self.x[0:3, 3]
        x_error[3:6] = get_phi(self.x_desired[0:3, 0:3], self.x[0:3, 0:3])

        xdot_error = self.xdot_desired - self.xdot

        J = self.robot_data.getJacobian()
        J_pinv = np.linalg.pinv(J)

        self.qdot_desired = J_pinv @ (kp* (x_error) + kd * (xdot_error))
        return self.q + self.dt * self.qdot_desired
     
    def PDControl(self, q_desired:np.ndarray, qdot_desired:np.ndarray, kp:float, kd:float) -> np.ndarray:
        q_error = q_desired - self.q
        qdot_error = qdot_desired - self.qdot
        tau = self.robot_data.getMassMatrix() @ (kp * q_error + kd * qdot_error) + self.robot_data.getCoriolis()
        return tau
    
    def targetPoseCallback(self, msg: Pose):
        self.node.get_logger().info(f"[FR3Controller] Target pose received: {msg}")
        self.is_target_pose_changed = True
        # Convert the Pose message to a 4x4 transformation matrix
        quat = [msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w]

        T = np.eye(4)
        T[:3, :3] = R.from_quat(quat).as_matrix()
        T[:3, 3] = [msg.position.x,
                    msg.position.y,
                    msg.position.z]
        
        self.target_pose = T
        
    def pub_EEpose_callback(self):
        # Publish end-effector pose
        ee_pose_msg = Pose()
        ee_pose_msg.position.x = self.robot_data.getPose()[0, 3]
        ee_pose_msg.position.y = self.robot_data.getPose()[1, 3]
        ee_pose_msg.position.z = self.robot_data.getPose()[2, 3]
        
        quat = R.from_matrix(self.robot_data.getPose()[0:3, 0:3]).as_quat()
        ee_pose_msg.orientation.x = quat[0]
        ee_pose_msg.orientation.y = quat[1]
        ee_pose_msg.orientation.z = quat[2]
        ee_pose_msg.orientation.w = quat[3]
        
        self.EEpose_pub.publish(ee_pose_msg)
        
