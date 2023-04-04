from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_wp_nav',
            executable='run.py',
            parameters=[{'waypoint_file' : '/home/kazuki/ros2_ws/src/test_wp_nav/waypoints/waypoints.yaml',
                         'dist_err' : 0.6,
                         'loop' : True}]
        ),
    ])