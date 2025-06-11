import numpy as np
import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from dyros_robot_controller_wrapper_cpp import FR3RobotData as RobotDatacpp

"""
FR3 URDF Joint Information
id | name              | nq | nv | idx_q | idx_v
----+------------------+----+----+-------+------
1  |        fr3_joint1 |  1 |  1 |     0 |    0
2  |        fr3_joint2 |  1 |  1 |     1 |    1
3  |        fr3_joint3 |  1 |  1 |     2 |    2
4  |        fr3_joint4 |  1 |  1 |     3 |    3
5  |        fr3_joint5 |  1 |  1 |     4 |    4
6  |        fr3_joint6 |  1 |  1 |     5 |    5
7  |        fr3_joint7 |  1 |  1 |     6 |    6
"""

class FR3RobotData():
    def __init__(self, node: Node, mj_joint_names: list):
        self.node = node
        self.mj_joint_names = mj_joint_names
        
        urdf_path = os.path.join(
            get_package_share_directory('dyros_robot_controller'),
            'robot',
            'fr3.urdf'
        )
        self.robot_data = RobotDatacpp(urdf_path, True)
        
        self.rd_joint_names = self.robot_data.getJointNames()
        self.dof = len(self.rd_joint_names)
        
        self.q = np.zeros(self.dof)
        self.qdot = np.zeros(self.dof)
        self.tau_ext = np.zeros(self.dof)
    
    def updateState(self, 
                    pos_dict: dict, 
                    vel_dict: dict, 
                    tau_ext_dict: dict, 
                    sensor_dict:dict):
        for rd_joint_name in self.rd_joint_names:
            rd_index = self.rd_joint_names.index(rd_joint_name)
            
            self.q[rd_index] = pos_dict[rd_joint_name]
            self.qdot[rd_index] = vel_dict[rd_joint_name]
            self.tau_ext[rd_index] = tau_ext_dict[rd_joint_name]
        
        if(not self.robot_data.updateState(self.q, self.qdot)):
            self.node.get_logger().error("[FR3RobotData] Failed to update robot state.")

# ==================================================================================================

    def getPose(self, link_name: str="fr3_link8"):
        return self.robot_data.getPose(link_name)
    
    def getJacobian(self, link_name: str="fr3_link8"):
        return self.robot_data.getJacobian(link_name)
    
    def getVelocity(self, link_name: str="fr3_link8"):
        return self.robot_data.getVelocity(link_name)
    
    def getMassMatrix(self) -> np.ndarray:
        return self.robot_data.getMassMatrix()
    
    def getCoriolis(self) -> np.ndarray:
        return self.robot_data.getCoriolis()

    def getGravity(self) -> np.ndarray:
        return self.robot_data.getGravity()
    
    def getNonlinearEffects(self) -> np.ndarray:
        return self.robot_data.getNonlinearEffects()