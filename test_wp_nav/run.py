#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math
import tf_transformations
from tf_transformations import euler_from_quaternion

class test_wp_nav(Node):
    def __init__(self):
        super().__init__('test_wp_nav')
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.start_srv = self.create_service(Trigger, '/start_wp_nav', self.start_callback)
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 1)
        self.pose_x, self.pose_y = 0.0, 0.0
        self.dist_err = 0.4
        self.wp_num = 0
        self.waypoint_file = '/home/kazuki/ros2_ws/src/test_wp_nav/waypoints/waypoints.yaml'
        with open(self.waypoint_file, 'r') as yml:
            self.waypoint = yaml.safe_load(yml)

    def start_callback(self, request, response):
        self.get_logger().info('Start waypoint navigation')
        response.success = True
        response.message = 'Start succeeded'
        return response
    
    def pose_callback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
    
    def send_goal(self, wp_num, orientation):
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for action server')
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.pose.position.x = self.waypoint['waypoints'][wp_num]['point']['x']
        goal_pose.pose.pose.position.y = self.waypoint['waypoints'][wp_num]['point']['y']
        goal_pose.pose.pose.position.z = self.waypoint['waypoints'][wp_num]['point']['z']
        goal_pose.pose.pose.orientation.x = orientation[0]
        goal_pose.pose.pose.orientation.y = orientation[1]
        goal_pose.pose.pose.orientation.z = orientation[2]
        goal_pose.pose.pose.orientation.w = orientation[3]
        goal = self.nav_client.send_goal_async(goal_pose)
    
    def calc_deg(self, wp_num):
        pose_x = self.pose_x
        pose_y = self.pose_y
        goal_x = self.waypoint['waypoints'][wp_num]['point']['x']
        goal_y = self.waypoint['waypoints'][wp_num]['point']['y']
        radian = math.atan2(goal_y - pose_y, goal_x - pose_x)
        orientation = tf_transformations.quaternion_from_euler(0, 0, radian)
        return orientation
    
    def check_distance(self, pose_x, pose_y):
        goal_x = self.waypoint['waypoints'][self.wp_num]['point']['x']
        goal_y = self.waypoint['waypoints'][self.wp_num]['point']['y']
        dist = math.sqrt((goal_x - pose_x)**2 + (goal_y - pose_y)**2)
        self.get_logger().info('Distance to goal: %f' % dist)
        return dist

    def status(self, dist):
        if dist < self.dist_err:
            self.get_logger().info('Goal succeeded!')
            self.wp_num += 1
        else:
            self.get_logger().info('Moving to goal')
        
    def navigation(self):
        if self.wp_num < len(self.waypoint['waypoints']):
            orientation = self.calc_deg(self.wp_num)
            self.send_goal(self.wp_num, orientation)
            dist = self.check_distance(self.pose_x, self.pose_y)
            self.status(dist)
        else:
            self.get_logger().info('Finish navigation')
            self.wp_num = 0

def main(args=None):
    rclpy.init(args=args)
    wp_nav = test_wp_nav()
    while rclpy.ok():
        rclpy.spin_once(wp_nav)
        wp_nav.navigation()
        time.sleep(0.5)

if __name__ == '__main__':
    main()