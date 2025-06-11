import numpy as np
import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from dyros_robot_controller_wrapper_cpp import HuskyFR3RobotData as RobotDatacpp
from scipy.spatial.transform import Rotation as R
from dyros_robot_controller.utils import LowPassFilter
"""
HuskyFR3 URDF Joint Information
id | name              | nq | nv | idx_q | idx_v
----+-------------------+----+----+-------+------
1  |        root_joint |  4 |  3 |     0 |    0
2  |        fr3_joint1 |  1 |  1 |     4 |    3
3  |        fr3_joint2 |  1 |  1 |     5 |    4
4  |        fr3_joint3 |  1 |  1 |     6 |    5
5  |        fr3_joint4 |  1 |  1 |     7 |    6
6  |        fr3_joint5 |  1 |  1 |     8 |    7
7  |        fr3_joint6 |  1 |  1 |     9 |    8
8  |        fr3_joint7 |  1 |  1 |    10 |    9
9  |        left_wheel |  1 |  1 |    11 |   10
10 |       right_wheel |  1 |  1 |    12 |   11

HuskyFR3 MuJoCo Joint/Sensor Information
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


class HuskyFR3RobotData():
    def __init__(self, node: Node, mj_joint_names: list):
        self.node = node
        self.mj_joint_names = mj_joint_names
        
        urdf_path = os.path.join(
            get_package_share_directory('dyros_robot_controller'),
            'robot',
            'husky_fr3.urdf'
        )
        self.robot_data = RobotDatacpp(urdf_path, True)
        
        
        self.rd_joint_names = self.robot_data.getJointNames()
        

        self.q_virtual = np.zeros(3)    # (x, y, yaw)
        self.qdot_virtual = np.zeros(3) # (vx, vy, yaw_rate)
        
        self.q_mobile = np.zeros(2)     # (left_wheel, right_wheel)
        self.qdot_mobile = np.zeros(2)  # (left_wheel, right_wheel)
        
        self.q_mani = np.zeros(7)       # (fr3_joint1, ..., fr3_joint7)
        self.qdot_mani = np.zeros(7)    # (fr3_joint1, ..., fr3_joint7)
        self.tau_ext_mani = np.zeros(7) # (fr3_joint1, ..., fr3_joint7)
    
    def updateState(self, 
                    pos_dict: dict, 
                    vel_dict: dict, 
                    tau_ext_dict: dict, 
                    sensor_dict:dict):
        self.q_virtual[:2] = LowPassFilter(sensor_dict["position_sensor"][:2], self.q_virtual[:2], 1000, 10)
        self.q_virtual[2] = R.from_quat(sensor_dict["orientation_sensor"], scalar_first=True).as_euler('zyx', degrees=False)[2]
        self.qdot_virtual[:2] = sensor_dict["linear_velocity_sensor"][:2]
        self.qdot_virtual[2] = sensor_dict["angular_velocity_sensor"][2]
        self.q_virtual_tmp = np.concatenate([self.q_virtual[:2], 
                                             np.array([np.cos(self.q_virtual[2]),
                                                       np.sin(self.q_virtual[2])])]) # (x, y, cos(yaw), sin(yaw))
        
        self.qdot_mobile[0] = vel_dict["front_left_wheel"]
        self.qdot_mobile[1] = vel_dict["front_right_wheel"]
        
        for joint_inx in range(7):
            self.q_mani[joint_inx] =       pos_dict["fr3_joint"+str(joint_inx+1)]
            self.qdot_mani[joint_inx] =    vel_dict["fr3_joint"+str(joint_inx+1)]
            self.tau_ext_mani[joint_inx] = tau_ext_dict["fr3_joint"+str(joint_inx+1)]
        
        # Update the robot_data instance (C++ wrapper) with the new state arrays.
        if(not self.robot_data.updateState(np.concatenate([self.q_virtual_tmp, self.q_mani,    self.q_mobile]),
                                           np.concatenate([self.qdot_virtual,  self.qdot_mani, self.qdot_mobile])
                                           )):
            self.node.get_logger().info("[HuskyFR3RobotData] Failed to update robot state.")

    # ==================================================================================================

    def getPose(self, link_name: str="fr3_link8"):
        return self.robot_data.getPose(link_name)
    
    def getJacobian(self, link_name: str="fr3_link8"):
        return self.robot_data.getJacobianActuated(link_name)
    
    def getVelocity(self, link_name: str="fr3_link8"):
        return self.robot_data.getVelocity(link_name)
    
    def getMassMatrix(self) -> np.ndarray:
        return self.robot_data.getMassMatrixActuated()
    
    def getCoriolis(self) -> np.ndarray:
        return self.robot_data.getCoriolisActuated()

    def getGravity(self) -> np.ndarray:
        return self.robot_data.getGravityActuated()
    
    def getNonlinearEffects(self) -> np.ndarray:
        return self.robot_data.getNonlinearEffectsActuated()