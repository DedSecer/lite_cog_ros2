#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import yaml
import os
import time

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Parameters
        self.declare_parameter('waypoints_file', '')
        
        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        
        if not waypoints_file or not os.path.exists(waypoints_file):
            self.get_logger().error(f"Waypoints file not found: {waypoints_file}")
            return

        with open(waypoints_file, 'r') as f:
            config = yaml.safe_load(f)
            self.waypoints = config.get('waypoints', [])

        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints.")
        
        # Status Publisher for Supervisor
        self.status_pub = self.create_publisher(String, '/nav_mission_status', 10)
        
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_waypoint_idx = 0
        
        # Wait for action server
        self.get_logger().info("Waiting for navigate_to_pose action server...")
        self._action_client.wait_for_server()
        
        # Start navigation
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info("All waypoints reached! Sending completion signal...")
            msg = String()
            msg.data = "COMPLETED"
            self.status_pub.publish(msg)
            # Stay alive for a bit to ensure message is sent
            time.sleep(2.0)
            return

        wp = self.waypoints[self.current_waypoint_idx]
        self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}")
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(wp['x'])
        goal_msg.pose.pose.position.y = float(wp['y'])
        goal_msg.pose.pose.position.z = float(wp.get('z', 0.0))
        goal_msg.pose.pose.orientation.x = float(wp.get('ox', 0.0))
        goal_msg.pose.pose.orientation.y = float(wp.get('oy', 0.0))
        goal_msg.pose.pose.orientation.z = float(wp.get('oz', 0.0))
        goal_msg.pose.pose.orientation.w = float(wp.get('ow', 1.0))

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Waypoint {self.current_waypoint_idx + 1} reached.")
        self.current_waypoint_idx += 1
        time.sleep(1.0)
        self.send_next_waypoint()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
