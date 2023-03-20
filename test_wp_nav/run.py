#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import yaml
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class test_wp_nav(Node):
    def __init__(self):
        super().__init__('test_wp_nav')
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.start_srv = self.create_service(Trigger, '/start_wp_nav', self.start_callback)
        self.wp_num = 0
        self.waypoint_file = '/home/kazuki/test_ws/src/test_wp_nav/waypoints/waypoints.yaml'
        with open(self.waypoint_file, 'r') as yml:
            self.waypoint = yaml.safe_load(yml)
        self.flag = False

    def start_callback(self, request, response):
        self.get_logger().info('Start waypoint navigation')
        response.success = True
        response.message = 'Start succeeded'
        self.flag = True
        return response
    
    def send_goal(self, waypoint, wp_num):
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for action server')
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.pose.position.x = waypoint['waypoints'][wp_num]['point']['x']
        goal_pose.pose.pose.position.y = waypoint['waypoints'][wp_num]['point']['y']
        goal_pose.pose.pose.position.z = waypoint['waypoints'][wp_num]['point']['z']
        goal_pose.pose.pose.orientation.x = 0.0
        goal_pose.pose.pose.orientation.y = 0.0
        goal_pose.pose.pose.orientation.z = 0.0
        goal_pose.pose.pose.orientation.w = 0.1
        goal = self.nav_client.send_goal_async(goal_pose)
        return goal

    def status(self, goal):
        rclpy.spin_until_future_complete(self, goal)
        goal_handle = goal.result()
        status = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, status)

        if status.result() is not None:
            if status.result().status == GoalStatus.STATUS_ACCEPTED:
                self.get_logger().info('Goal accepted!')
            elif status.result().status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Goal succeeded!')
                self.wp_num += 1
            elif status.result().status == GoalStatus.STATUS_CANCELING:
                self.get_logger().info('Goal canceling...')
            elif status.result().status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Goal canceled!')
            elif status.result().status == GoalStatus.STATUS_EXECUTING:
                self.get_logger().info('Goal executing...')
            elif status.result().status == GoalStatus.STATUS_ABORTED:
                self.get_logger().info('Goal aborted!')
            elif status.result().status == GoalStatus.STATUS_UNKNOWN:
                self.get_logger().info('Unknown status')
        
    def navigation(self):
        if self.flag:
            if self.wp_num < len(self.waypoint['waypoints']):
                goal = self.send_goal(self.waypoint, self.wp_num)
                self.status(goal)
            else:
                self.get_logger().info('Finish navigation')

def main(args=None):
    rclpy.init(args=args)
    wp_nav = test_wp_nav()
    rclpy.spin_once(wp_nav)
    while rclpy.ok():
        wp_nav.navigation()
        time.sleep(1)

if __name__ == '__main__':
    main()