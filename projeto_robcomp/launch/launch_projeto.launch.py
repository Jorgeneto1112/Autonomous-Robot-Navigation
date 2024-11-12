from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robcomp_util',
            executable='mobilenet_detector',
            output='screen'),
        Node(
            package='robcomp_util',
            executable='creeper_pub',
            output='screen'),
        Node(
            package='robcomp_util',
            executable='aruco_detector',
            output='screen'),
    ],)