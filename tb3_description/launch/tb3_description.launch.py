#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Define file names
    xacro_file_name = 'turtlebot3_waffle_base.urdf.xacro'
    urdf_file_name = 'turtlebot3_waffle_base.urdf'

    # Get package share directory
    tb3_description_dir = get_package_share_directory('tb3_description')

    # Define full paths to the URDF and XACRO files
    urdf_file = os.path.join(tb3_description_dir, 'urdf', urdf_file_name)
    xacro_file = os.path.join(tb3_description_dir, 'urdf', xacro_file_name)

    # Process the XACRO file
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    # Write the URDF file
    with open(urdf_file, 'w') as f:
        f.write(robot_desc)

    # Return LaunchDescription with nodes
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_file],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_pole_tf',
            arguments=['-0.22', '0.06', '0.038', '0.0', '0.0', '0.0', 'base_link', 'pole_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_laser_tf',
            arguments=['0.0', '0.0', '0.02', '0.0', '0.0', '0.0', 'pole_link', 'flag_link'],
        ),
    ])
