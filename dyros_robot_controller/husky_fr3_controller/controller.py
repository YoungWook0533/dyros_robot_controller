import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import threading
import time
from scipy.spatial.transform import Rotation as R

from mujoco_ros_sim import ControllerInterface
from .robot_data import HuskyFR3RobotData

from dyros_robot_controller_wrapper_cpp import HuskyFR3Controller as Controllercpp

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from srmt2.planning_scene import PlanningScene
from srmt2.planner.rrt_connect import SuhanRRTConnect
from srmt2.kinematics import TRACIK

"""
HuskyFR3 MuJoCo Joint/Sensor Imformation
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  0 |                      | _Free  |  7 |  6 |     0 |    0
  1 | front_left_wheel     | _Hinge |  1 |  1 |     7 |    6
  2 | front_right_wheel    | _Hinge |  1 |  1 |     8 |    7
  3 | rear_left_wheel      | _Hinge |  1 |  1 |     9 |    8
  4 | rear_right_wheel     | _Hinge |  1 |  1 |    10 |    9
  5 | fr3_joint1           | _Hinge |  1 |  1 |    11 |   10
  6 | fr3_joint2           | _Hinge |  1 |  1 |    12 |   11
  7 | fr3_joint3           | _Hinge |  1 |  1 |    13 |   12
  8 | fr3_joint4           | _Hinge |  1 |  1 |    14 |   13
  9 | fr3_joint5           | _Hinge |  1 |  1 |    15 |   14
 10 | fr3_joint6           | _Hinge |  1 |  1 |    16 |   15
 11 | fr3_joint7           | _Hinge |  1 |  1 |    17 |   16

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | left_wheel           | _Joint  | front_left_wheel
  1 | right_wheel          | _Joint  | front_right_wheel
  2 | fr3_joint1           | _Joint  | fr3_joint1
  3 | fr3_joint2           | _Joint  | fr3_joint2
  4 | fr3_joint3           | _Joint  | fr3_joint3
  5 | fr3_joint4           | _Joint  | fr3_joint4
  6 | fr3_joint5           | _Joint  | fr3_joint5
  7 | fr3_joint6           | _Joint  | fr3_joint6
  8 | fr3_joint7           | _Joint  | fr3_joint7

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
  0 | position_sensor             | Framepos         |   3 |   0 | Site:husky_site
  1 | orientation_sensor          | Framequat        |   4 |   3 | Site:husky_site
  2 | linear_velocity_sensor      | Framelinvel      |   3 |   7 | Site:husky_site
  3 | angular_velocity_sensor     | Frameangvel      |   3 |  10 | Site:husky_site

"""
class HuskyFR3Controller(ControllerInterface):
    def __init__(self, node: Node, dt: float, mj_joint_dict: list):
        super().__init__(node, dt, mj_joint_dict)
        
        self.robot_data = HuskyFR3RobotData(self.node, mj_joint_dict["joint_names"])
        self.controller = Controllercpp(self.dt, self.robot_data.robot_data)
        
        self.key_sub = self.node.create_subscription(Int32, 'husky_fr3_controller/mode_input', self.keyCallback, 10)
        self.target_pose_sub = self.node.create_subscription(Pose, 'husky_fr3_controller/target_pose', self.targetPoseCallback, 10)
        
        self.EEpose_pub = self.node.create_publisher(Pose, 'husky_fr3_controller/ee_pose', 10)
        
        urdf_path = os.path.join(
            get_package_share_directory('dyros_robot_controller'),
            'robot',
            'husky_fr3.urdf'
        )
                
        self.compute_slow_thread = threading.Thread(target=self.computeSlow)
        
    def starting(self) -> None:
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
        self.controller.updateState(self.current_time)
                        
    def compute(self) -> None:
        self.controller.computeFast()
        
    def computeSlow(self) -> None:
        while rclpy.ok:
            self.controller.computeSlow()
            time.sleep(0.001)
        
    def getCtrlInput(self) -> dict:
        ctrl = np.array(self.controller.getCtrlInput())
        ctrl_dict = {"fr3_joint1": ctrl[0],
                     "fr3_joint2": ctrl[1],
                     "fr3_joint3": ctrl[2],
                     "fr3_joint4": ctrl[3],
                     "fr3_joint5": ctrl[4],
                     "fr3_joint6": ctrl[5],
                     "fr3_joint7": ctrl[6],
                     "left_wheel": ctrl[7],
                     "right_wheel": ctrl[8],
                    }
        return ctrl_dict
    
    # =============================================================================================
    def keyCallback(self, msg: Int32):
        self.node.get_logger().info(f"[HuskyFR3Controller] Key input received: {msg.data}") 
        if msg.data == 1:
            self.controller.setMode("home")
        elif msg.data == 2:
            self.controller.setMode("wholebody_grav_comp")
        elif msg.data == 3:
            self.controller.setMode("wholebody_impedence")

    def targetPoseCallback(self, msg: Pose):
        self.node.get_logger().info(f"[HuskyFR3Controller] Target pose received: {msg}")
        self.is_target_pose_changed = True
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