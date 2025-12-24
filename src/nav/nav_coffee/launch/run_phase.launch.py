
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    # Get the requested phase name
    phase_name = LaunchConfiguration('phase_name').perform(context)
    
    # Locate the config file
    pkg_share = get_package_share_directory('nav_coffee')
    config_path = os.path.join(pkg_share, 'config', 'phases_config.yaml')
    
    if not os.path.exists(config_path):
        print(f"[ERROR] Config file not found at {config_path}")
        return []
        
    try:
        # Read the configuration
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            phases = config.get('phases', {})
            
        if phase_name not in phases:
             available = ', '.join(phases.keys())
             print(f"[ERROR] Phase '{phase_name}' not found in configuration. Available phases: {available}")
             return []

        # Extract phase-specific parameters
        phase_config = phases[phase_name]
        map_file = phase_config['map_file']
        pcd_file = phase_config['pcd_file']
        
        print(f"[INFO] Launching Phase: {phase_name}")
        print(f"[INFO] Map File: {map_file}")
        print(f"[INFO] PCD File: {pcd_file}")

        # Path to the main navigation launch file
        launch_file_path = os.path.join(pkg_share, 'launch', 'coffee_nav.launch.py')
        
        # Include the launch file with the appropriate arguments
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file_path),
                launch_arguments={
                    'map_server_config_file': map_file,
                    'globalmap_pcd': pcd_file,
                    'phases_config_file': config_path,
                    'current_phase': phase_name
                }.items()
            )
        ]

    except Exception as e:
        print(f"[ERROR] failed to parse config or launch: {e}")
        return []

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'phase_name',
            default_value='phase_1',
            description='Name of the phase to launch (e.g., phase_1, phase_2)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
