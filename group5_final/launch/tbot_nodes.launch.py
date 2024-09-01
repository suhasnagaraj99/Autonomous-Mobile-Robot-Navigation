"""
Launch file for the tbot nodes
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    camera1_broadcaster = Node(
        package="group5_final",
        executable="camera1_broadcaster",
        parameters=[{'use_sim_time': True}]
    )
    camera2_broadcaster = Node(
        package="group5_final",
        executable="camera2_broadcaster",
        parameters=[{'use_sim_time': True}]
    )  
    camera3_broadcaster = Node(
        package="group5_final",
        executable="camera3_broadcaster",
        parameters=[{'use_sim_time': True}]
    )
    camera4_broadcaster = Node(
        package="group5_final",
        executable="camera4_broadcaster",
        parameters=[{'use_sim_time': True}]
    )    
    camera5_broadcaster = Node(
        package="group5_final",
        executable="camera5_broadcaster",
        parameters=[{'use_sim_time': True}]
    )    
    ld = LaunchDescription()
    ld.add_action(camera1_broadcaster)
    ld.add_action(camera2_broadcaster)
    ld.add_action(camera3_broadcaster)
    ld.add_action(camera4_broadcaster)
    ld.add_action(camera5_broadcaster)
    return ld
