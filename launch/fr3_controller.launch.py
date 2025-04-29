from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    """
    fr3_controller.launch.py:
    - Executes the 'mujoco_ros_sim' node from the mujoco_ros_sim package.
    - Sets the 'controller_class' parameter (and optionally 'robot_data_class')
      with the default value 'fr3_controller.controller.FR3Controller'
    - Accepts a 'robot_name' launch argument to specify the robot model used in MuJoCo.
    """

    # Optional robot name parameter
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='franka_fr3_torque',
        description='Name of the robot model used in MuJoCo'
    )

    # Python path for the custom controller class
    controller_class_arg = DeclareLaunchArgument(
        'controller_class',
        default_value='fr3_controller.controller.FR3Controller',
        description='Python path for FR3Controller'
    )

    # Launch configurations for the parameters
    robot_name = LaunchConfiguration('robot_name')
    controller_class = LaunchConfiguration('controller_class')

    # Node to run the mujoco_ros_sim simulation
    sim_node = Node(
        package='mujoco_ros_sim',          # Package name (see setup.py and package.xml)
        executable='mujoco_ros_sim',       # Entry point: main function in mujoco_ros_sim.py
        name='mujoco_sim_node',
        output='screen',
        parameters=[
            {'robot_name': robot_name},
            {'controller_class': controller_class},
        ]
    )

    return LaunchDescription([
        robot_name_arg,
        controller_class_arg,
        sim_node,
    ])
