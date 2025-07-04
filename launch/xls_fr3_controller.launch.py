from launch import LaunchDescription
from launch import actions, substitutions
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("dyros_robot_controller"),
        "config",
        "teleop_twist_joy.yaml"
    )
    
    declare_args = [
        actions.DeclareLaunchArgument('joy_vel',
            default_value=TextSubstitution(text='cmd_vel')),
        actions.DeclareLaunchArgument('publish_stamped_twist',
            default_value=TextSubstitution(text='false')),
        actions.DeclareLaunchArgument('joy_dev',
            default_value=TextSubstitution(text='0')),
    ]
    
    sim_node = Node(
    package='mujoco_ros_sim',
    executable='mujoco_ros_sim',
    name='mujoco_sim_node',
    output='screen',
    parameters=[
        {'robot_name': 'fr3_xls'},
        {'controller_class': 'dyros_robot_controller.XLSFR3Controller'},
        ],
    remappings=[('/joint_states', '/joint_states_raw')],
    )

    urdf_path = os.path.join(get_package_share_directory('dyros_robot_controller'),
                            'robot', 'xls_fr3.urdf')
    srdf_path = os.path.join(get_package_share_directory('dyros_robot_controller'),
                            'robot', 'xls_fr3.srdf')
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
    with open(srdf_path, 'r') as infp:
        robot_description_semantic = infp.read()
        
    rviz_config_file = os.path.join(
        get_package_share_directory("dyros_robot_controller"),
        "launch", 
        "xls_fr3_rviz.rviz"
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    gui_node = Node(
        package='dyros_robot_controller',
        executable='XLSFR3ControllerQT',
        name='XLSFR3ControllerQT',
        output='screen',
        emulate_tty=True,
    )
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }],
    )
    
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[
            config_file,
            {'publish_stamped_twist': False},
        ],
        remappings=[
            # ('cmd_vel', joy_vel),
            ('cmd_vel', '/xml_fr3_controller/cmd_vel'),
        ],
    )

    return LaunchDescription([
        sim_node,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node,
        gui_node,
        joy_node,
        teleop_twist_joy_node,
    ])