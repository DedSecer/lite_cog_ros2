#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import signal
import time

class NavSupervisor(Node):
    def __init__(self):
        super().__init__('nav_supervisor')
        
        # Phase parameters
        self.phase = 1
        self.current_process = None
        
        # Config for phases
        self.phases = {
            1: {
                'map': '/home/ysc/lite_cog_ros2/system/map/lite3.yaml',
                'pcd': '/home/ysc/lite_cog_ros2/system/map/lite3.pcd'
            },
            2: {
                'map': '/home/ysc/lite_cog_ros2/system/map/next_phase.yaml',
                'pcd': '/home/ysc/lite_cog_ros2/system/map/next_phase.pcd'
            }
        }
        
        self.subscription = self.create_subscription(
            String,
            '/nav_mission_status',
            self.status_callback,
            10
        )
        
        self.get_logger().info("Nav Supervisor started.")
        self.start_phase(self.phase)

    def start_phase(self, phase_num):
        if phase_num not in self.phases:
            self.get_logger().info("All phases completed. Exiting supervisor.")
            rclpy.shutdown()
            return

        config = self.phases[phase_num]
        self.get_logger().info(f"--- STARTING PHASE {phase_num} ---")
        
        cmd = [
            'ros2', 'launch', 'nav_coffee', 'coffee_nav.launch.py',
            f'map_server_config_file:={config["map"]}',
            f'globalmap_pcd:={config["pcd"]}'
        ]
        
        # Use start_new_session=True to create a new process group for easy cleanup
        self.current_process = subprocess.Popen(
            cmd,
            preexec_fn=os.setsid
        )
        self.get_logger().info(f"Started launch process with PID: {self.current_process.pid}")

    def stop_current_phase(self):
        if self.current_process:
            self.get_logger().warn(f"Stopping Phase {self.phase} (Killing process group)...")
            try:
                # Kill the entire process group (including all nodes started by launch)
                os.killpg(os.getpgid(self.current_process.pid), signal.SIGINT)
                self.current_process.wait(timeout=10)
            except Exception as e:
                self.get_logger().error(f"Error killing process: {e}")
                try:
                    os.killpg(os.getpgid(self.current_process.pid), signal.SIGKILL)
                except:
                    pass
            
            self.current_process = None
            self.get_logger().info("Process group terminated.")

    def status_callback(self, msg):
        if msg.data == "COMPLETED":
            self.get_logger().info(f"Phase {self.phase} mission completed signal received!")
            self.stop_current_phase()
            
            # Move to next phase
            self.phase += 1
            time.sleep(2.0) # Wait for cleanup
            self.start_phase(self.phase)

def main(args=None):
    rclpy.init(args=args)
    node = NavSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_current_phase()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

