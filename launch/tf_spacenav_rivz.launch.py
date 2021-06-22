#!/usr/bin/env python3
'''
FILE:   tf_spacenav_rviz.launch.py
DESC:   Launch file to start the tf_servo node, a simple example
        node that creates a single TF transform that is controllable
        via a 'delta_twist_cmds' published twist command.  This is 
        intended to test the function of the created tf_spacenav node, 
        which reads in the Spacemouse values, scales, and publishes a 
        twist command suitable for servoing.
DATE:   June 2021
AUTH:   scott@restfulrobotics.com
NOTE:

    Example use:

    $ cd ~/ros2_ws/src/tf_servo_pkg/launch
    $ ros2 launch tf_spacenav_rviz.launch.py
    
'''

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    # standard frames
    # https://www.ros.org/reps/rep-0105.html
    #static frame, world_tx_base_link
    static_frame_world_tx_base_link = Node(
        package= 'tf2_ros', 
        executable= 'static_transform_publisher',
        name='world_tx_base_link',
        output='screen',
        arguments = ['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )

    #launch tf_servo
    tf_servo_node = Node(
        package='tf_servo_pkg',
        executable='tf_servo',
        name='tf_servo_node'
    )

    #spacenav_control
    tf_servo_spacenav = Node(
        package='tf_servo_pkg',
        executable='tf_spacenav',
        name='tf_servo_spacenav'
    )


    rviz_path = '/home/snortman/ros2_ws/src/tf_servo_pkg/launch/world_tx_base_link.rviz'
    #Launch RVIZ2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=[
            '-d', rviz_path
        ]
    )

    ld.add_action(static_frame_world_tx_base_link)
    ld.add_action(tf_servo_node)
    ld.add_action(tf_servo_spacenav)
    ld.add_action(rviz2_node)

    return ld



