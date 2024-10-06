import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    urdf_description_file = os.path.join(get_package_share_directory(
        "dynaarm_standalone_description"), "resource/xacro/dynaarm_standalone.urdf.xacro")
    urdf = Command(['xacro ', urdf_description_file])
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(urdf, value_type=str)}])
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )
    rviz_config_file_path = os.path.join(get_package_share_directory("dynaarm_standalone_description"),'launch/config.rviz')
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='both',
        emulate_tty=True,
        arguments=['-d', rviz_config_file_path],
    )

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
    return ld