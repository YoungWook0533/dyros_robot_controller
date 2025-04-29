import numpy as np
import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from fr3_controller_wrapper_cpp import RobotData as RobotDatacpp

class FR3RobotData():
    """
    Concrete implementation of robot state data for the FR3 robot.
    
    This class loads robot-specific data from a URDF file and uses a C++ wrapper
    (RobotDatacpp) to handle the robot's state. It provides an interface to update
    the robot's state with simulation data (positions, velocities, torques) from a MuJoCo model.
    
    Attributes:
        node (Node): ROS2 node instance used for logging and communication.
        mj_joint_names (list): List of joint names from the MuJoCo simulation.
        robot_data (RobotDatacpp): Instance of the C++ robot data wrapper.
        rb_joint_names (list): List of joint names as defined in the robot_data wrapper.
        q (np.ndarray): Array for storing joint positions.
        qdot (np.ndarray): Array for storing joint velocities.
        tau_ext (np.ndarray): Array for storing joint torques.
    """

    def __init__(self, node: Node, mj_joint_names: list):
        """
        Initializes the FR3RobotData object.
        
        This method sets up the robot data by loading the FR3 URDF file from the 
        'dyros_robot_controller' package share directory, initializes the C++ robot
        data wrapper, and creates arrays to store the robot's state.
        
        Parameters:
            node (Node): A ROS2 node instance used for logging and communication.
            mj_joint_names (list): A list of joint names (strings) from the MuJoCo simulation.
            
        Returns:
            None
        """
        self.node = node
        self.mj_joint_names = mj_joint_names
        
        # Construct the path to the FR3 URDF file using the package share directory.
        urdf_path = os.path.join(
            get_package_share_directory('dyros_robot_controller'),
            'fr3_controller',
            'robot',
            'fr3.urdf'
        )
        # Create the C++ robot data instance using the URDF file.
        self.robot_data = RobotDatacpp(urdf_path)
        
        # Retrieve the robot joint names from the C++ robot data wrapper.
        self.rb_joint_names = self.robot_data.getJointNames()
        
        # Initialize numpy arrays for joint positions (q), velocities (qdot), and torques (tau_ext)
        # with a size corresponding to the number of robot joints.
        self.q = np.zeros(len(self.rb_joint_names))
        self.qdot = np.zeros(len(self.rb_joint_names))
        self.tau_ext = np.zeros(len(self.rb_joint_names))
    
    def updateState(self, pos: np.ndarray, vel: np.ndarray, tau_ext: np.ndarray):
        """
        Updates the internal state of the FR3 robot data using simulation values.
        
        This method maps simulation state values from the MuJoCo model to the robot-specific
        joint ordering and then calls the C++ robot data wrapper to update its internal state.
        
        Parameters:
            pos (np.ndarray): Array containing current joint positions from the MuJoCo simulation.
            vel (np.ndarray): Array containing current joint velocities from the MuJoCo simulation.
            tau_ext (np.ndarray): Array containing current external joint torques from the MuJoCo simulation.
            
        Returns:
            None
        """
        # For each joint in the robot's joint list (as defined in the C++ wrapper)
        for rb_joint_name in self.rb_joint_names:
            # Find the index of the current robot joint name in the robot data array.
            rb_index = self.rb_joint_names.index(rb_joint_name)
            # Find the corresponding index in the MuJoCo joint names list.
            mj_index = self.mj_joint_names.index(rb_joint_name)
            
            # Map the simulation joint position to the robot data array.
            self.q[rb_index] = pos[mj_index]
            # Map the simulation joint velocity to the robot data array.
            self.qdot[rb_index] = vel[mj_index]
            # Map the simulation joint torque to the robot data array.
            self.tau_ext[rb_index] = tau_ext[mj_index]
        
        # Update the robot_data instance (C++ wrapper) with the new state arrays.
        if(not self.robot_data.updateState(self.q, self.qdot, self.tau_ext)):
            self.node.get_logger().error("[FR3RobotData] Failed to update robot state.")

# ==================================================================================================

    def getPose(self, link_name: str="fr3_link8"):
        return self.robot_data.getPose(link_name)
    
    def getJacobian(self, link_name: str="fr3_link8"):
        return self.robot_data.getJacobian(link_name)
    
    def getVelocity(self, link_name: str="fr3_link8"):
        return self.robot_data.getVelocity(link_name)
    
    def getMassMatrix(self) -> np.ndarray:
        """
        Retrieves the mass matrix from the robot data wrapper.
        
        Returns:
            np.ndarray: Array containing the mass matrix.
        """
        return self.robot_data.getMassMatrix()
    
    def getCoriolis(self) -> np.ndarray:
        """
        Retrieves the Coriolis vector from the robot data wrapper.
        
        Returns:
            np.ndarray: Array containing the Coriolis vector.
        """
        return self.robot_data.getCoriolis()

    def getGravity(self) -> np.ndarray:
        """
        Retrieves the gravity vector from the robot data wrapper.
        
        Returns:
            np.ndarray: Array containing the gravity vector.
        """
        return self.robot_data.getGravity()
    
    def getNonlinearEffects(self) -> np.ndarray:
        """
        Retrieves the nonlinear effects vector from the robot data wrapper.
        
        Returns:
            np.ndarray: Array containing the nonlinear effects vector.
        """
        return self.robot_data.getNonlinearEffects()