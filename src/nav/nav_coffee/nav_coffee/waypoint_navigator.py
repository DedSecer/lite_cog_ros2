#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import yaml
import os
import subprocess
import time

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Parameters
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('next_map_yaml', '/home/ysc/lite_cog_ros2/system/map/new_map.yaml')
        self.declare_parameter('next_map_pcd', '/home/ysc/lite_cog_ros2/system/map/new_map.pcd')
        
        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        
        if not waypoints_file or not os.path.exists(waypoints_file):
            self.get_logger().error(f"Waypoints file not found: {waypoints_file}")
            return

        with open(waypoints_file, 'r') as f:
            config = yaml.safe_load(f)
            self.waypoints = config.get('waypoints', [])

        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints.")
        
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_waypoint_idx = 0
        
        # Wait for action server
        self.get_logger().info("Waiting for navigate_to_pose action server...")
        self._action_client.wait_for_server()
        
        # Start navigation
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info("All waypoints reached! Executing restart logic...")
            self.restart_system()
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
        # Give a small pause between waypoints
        time.sleep(1.0)
        self.send_next_waypoint()

    def restart_system(self):
        next_map = self.get_parameter('next_map_yaml').get_parameter_value().string_value
        next_pcd = self.get_parameter('next_map_pcd').get_parameter_value().string_value
        
        self.get_logger().warn("SHUTTING DOWN AND RESTARTING WITH NEW MAP...")
        
        # Command to kill nodes and start new launch
        # We use a slight delay before launch to ensure ports/processes are freed
        cmd = (
            "pkill -f hdl_localization_composition; "
            "pkill -f bt_navigator; "
            "pkill -f controller_server; "
            "pkill -f planner_server; "
            "pkill -f map_server; "
            "pkill -f recover_server; "
            "pkill -f rviz2; "
            "pkill -f livox_ros_driver2_node; "
            "pkill -f lslidar_driver_node; "
            "sleep 3; "
            f"ros2 launch nav_coffee coffee_nav.launch.py map_server_config_file:={next_map} globalmap_pcd:={next_pcd}"
        )
        
        self.get_logger().info(f"Executing: {cmd}")
        
        # Use Popen so it doesn't block this process from exiting
        subprocess.Popen(['/bin/bash', '-c', cmd], preexec_fn=os.setsid)
        
        # Shutdown this node
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

