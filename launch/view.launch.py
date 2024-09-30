# Author: Timo Schwarzer
# Last-Updated: Mai 31, 2024
# Description: Visualize the Duatic Dynaarm

import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):

    robot = LaunchConfiguration("robot")    
    gui = LaunchConfiguration("gui")

    robot_value = robot.perform(context)    

    # Set the path to the URDF file
    if robot_value == "dynaarm":
        robot_type_xacro_file_name = f"dynaarm.xacro"

    # Load the robot description
    description_file_path = os.path.join(
        get_package_share_directory("dynaarm_description"),
        "urdf",        
        robot_type_xacro_file_name,
    )    
    robot_description_config = xacro.process_file(description_file_path)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_node = Node(
        condition=UnlessCondition(gui),
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    # A GUI to manipulate the joint state values
    start_joint_state_publisher_gui_node = Node(
        condition=IfCondition(gui),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Launch RViz
    rviz_node = Node(        
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("dynaarm_description"),
                "rviz",
                "config.rviz",
            ),
        ],
        parameters=[robot_description],
    )

    nodes_to_start = [
        start_joint_state_publisher_node,
        start_joint_state_publisher_gui_node,
        robot_state_pub_node,
        rviz_node,
    ]

    return nodes_to_start


def generate_launch_description():
    # Declare the launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            name="gui",
            default_value="True",
            description="Flag to enable joint_state_publisher_gui",
        )
    )    
    declared_arguments.append(
        DeclareLaunchArgument(
            name="robot",
            default_value="dynaarm",
            description="Select the desired duatic robot model",
        )
    )    

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
