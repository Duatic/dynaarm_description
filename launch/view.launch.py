import os
import xacro

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):

    dof = LaunchConfiguration("dof")    
    gui = LaunchConfiguration("gui")
    covers = LaunchConfiguration("covers")

    dof_value = dof.perform(context)
    covers_value = covers.perform(context)

    # Load the robot description
    pkg_share_description = FindPackageShare(package='dynaarm_description').find('dynaarm_description')
    doc = xacro.parse(open(os.path.join(pkg_share_description, 'xacro/dynaarm_standalone.urdf.xacro')))    
    xacro.process_doc(doc, mappings={'dof': dof_value, 'covers': covers_value})
    robot_description = {'robot_description': doc.toxml()}

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
                "launch",
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
            default_value="6dof",
            description="Select the desired degrees of freedom (dof)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="covers",
            default_value="false",
            description="Show or hide the covers of the robot",
        )
    )    

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
