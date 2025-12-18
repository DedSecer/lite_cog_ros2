#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import signal
import time
import yaml
from ament_index_python.packages import get_package_share_directory

class NavSupervisor(Node):
    def __init__(self):
        super().__init__('nav_supervisor')
        
        # Load phases configuration
        self.declare_parameter('phases_config_file', '')
        phases_config_file = self.get_parameter('phases_config_file').get_parameter_value().string_value
        
        if not phases_config_file:
            nav_coffee_dir = get_package_share_directory('nav_coffee')
            phases_config_file = os.path.join(nav_coffee_dir, 'config', 'phases_config.yaml')
        
        if not os.path.exists(phases_config_file):
            self.get_logger().error(f"Phases config file not found: {phases_config_file}")
            return
        
        with open(phases_config_file, 'r') as f:
            config = yaml.safe_load(f)
            self.phases_config = config.get('phases', {})
        
        # Get sorted phase names
        self.phase_names = sorted(self.phases_config.keys())
        self.current_phase_idx = 0
        self.current_process = None
        
        self.get_logger().info(f"Loaded {len(self.phase_names)} phases from config: {self.phase_names}")
        
        self.subscription = self.create_subscription(
            String,
            '/nav_mission_status',
            self.status_callback,
            10
        )
        
        self.get_logger().info("Nav Supervisor started.")
        self.start_current_phase()

    def start_current_phase(self):
        if self.current_phase_idx >= len(self.phase_names):
            self.get_logger().info("All phases completed. Exiting supervisor.")
            rclpy.shutdown()
            return

        phase_name = self.phase_names[self.current_phase_idx]
        phase_config = self.phases_config[phase_name]
        
        self.get_logger().info(f"--- STARTING {phase_name.upper()} ---")
        
        nav_coffee_dir = get_package_share_directory('nav_coffee')
        phases_config_file = os.path.join(nav_coffee_dir, 'config', 'phases_config.yaml')
        
        cmd = [
            'ros2', 'launch', 'nav_coffee', 'coffee_nav.launch.py',
            f'map_server_config_file:={phase_config["map_file"]}',
            f'globalmap_pcd:={phase_config["pcd_file"]}',
            f'phases_config_file:={phases_config_file}',
            f'current_phase:={phase_name}'
        ]
        
        # Use preexec_fn=os.setsid to create a new process group for easy cleanup
        self.current_process = subprocess.Popen(
            cmd,
            preexec_fn=os.setsid
        )
        self.get_logger().info(f"Started launch process with PID: {self.current_process.pid}")

    def stop_current_phase(self):
        if self.current_process:
            phase_name = self.phase_names[self.current_phase_idx] if self.current_phase_idx < len(self.phase_names) else "Unknown"
            self.get_logger().warn(f"Stopping {phase_name} (Killing process group)...")
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
            phase_name = self.phase_names[self.current_phase_idx] if self.current_phase_idx < len(self.phase_names) else "Unknown"
            self.get_logger().info(f"{phase_name} mission completed signal received!")
            self.stop_current_phase()
            
            # Move to next phase
            self.current_phase_idx += 1
            time.sleep(2.0) # Wait for cleanup
            self.start_current_phase()

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

