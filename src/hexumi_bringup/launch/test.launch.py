from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('hexumi_description'),
                             'urdf', 'hexumi.xacro')
    rviz_config_path = os.path.join(get_package_share_path('hexumi_description'),
                                    'rviz', 'config.rviz')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    
    global_parameters = os.path.join(
        get_package_share_directory('hexumi_bringup'),
        'config',
        'global_params.yaml'
    )
    
    global_param_node = Node(
        package='hexumi_bringup',
        executable='global_parameter_server',
        name='global_parameter_server',
        parameters=[global_parameters, {'robot_description': robot_description}]
    )
    
    teleop_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hexumi_bringup'),
                "launch/teleop.launch.py"
            )
        )
    )
    
    kinematics_node = Node(
        package='hexumi_kinematics',
        executable='hexumi_kinematics',
        name='hexumi_kinematics',
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{'source_list': ['/kinematics/joint_states']}]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        kinematics_node,
        teleop_launch_file,
        global_param_node,
        joint_state_publisher_node,
        rviz2_node
    ])
