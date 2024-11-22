# Copyright 2024 Duatic AG
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions, and
#    the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and
#    the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import xacro

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def string_to_bool(string_input):
    """Converts a string representation of a boolean value to a Python boolean.

    Args:
        string_input (str): The input string, e.g., "true", "false", "yes", "no".

    Returns:
        bool: The corresponding boolean value, or raises a ValueError for invalid input.
    """
    if string_input.lower() in ["true", "yes", "1"]:
        return True
    elif string_input.lower() in ["false", "no", "0"]:
        return False
    else:
        raise ValueError(f"Invalid boolean string: '{string_input}'")


def launch_setup(context, *args, **kwargs):

    dof = LaunchConfiguration("dof")
    gui = LaunchConfiguration("gui")
    covers = LaunchConfiguration("covers")
    version = LaunchConfiguration("version")
    dual = LaunchConfiguration("dual")

    dof_value = dof.perform(context)
    covers_value = covers.perform(context)
    version_value = version.perform(context)
    dual_value = dual.perform(context).strip().lower()

    # Load the robot description
    pkg_share_description = FindPackageShare(package="dynaarm_description").find("dynaarm_description")

    is_dual = string_to_bool(dual_value)
    if is_dual:
        doc = xacro.parse(open(os.path.join(pkg_share_description, "urdf/dynaarm_standalone_dual.urdf.xacro")))
    else:
        doc = xacro.parse(open(os.path.join(pkg_share_description, "urdf/dynaarm_standalone.urdf.xacro")))

    xacro.process_doc(
        doc,
        mappings={"dof": dof_value, "covers": covers_value, "version": version_value},
    )
    robot_description = {"robot_description": doc.toxml()}

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
                "config",
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
            name="dof",
            choices=["1", "2", "3", "4", "5", "6"],
            default_value="6",
            description="Select the desired degrees of freedom (dof)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="covers",
            default_value="False",
            description="Show or hide the covers of the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="version",
            default_value="v1",
            choices=["v1", "v2"],
            description="Select the desired version of robot ",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="dual",
            default_value="False",
            description="Select the desired version of robot ",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
