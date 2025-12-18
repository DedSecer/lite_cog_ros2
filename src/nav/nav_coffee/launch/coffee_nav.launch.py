import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    hdl_loc_dir = get_package_share_directory('hdl_localization')
    dr_nav2_dir = get_package_share_directory('dr_nav2')
    nav_coffee_dir = get_package_share_directory('nav_coffee')

    # Arguments
    map_server_config_file_arg = DeclareLaunchArgument(
        'map_server_config_file',
        default_value='/home/ysc/lite_cog_ros2/system/map_inner/lite3.yaml',
        description='Full path to the map yaml file'
    )

    globalmap_pcd_arg = DeclareLaunchArgument(
        'globalmap_pcd',
        default_value='/home/ysc/lite_cog_ros2/system/map_inner/lite3.pcd',
        description='Full path to the global map PCD file'
    )

    phases_config_file_arg = DeclareLaunchArgument(
        'phases_config_file',
        default_value=os.path.join(nav_coffee_dir, 'config', 'phases_config.yaml'),
        description='Full path to the phases configuration yaml file'
    )

    current_phase_arg = DeclareLaunchArgument(
        'current_phase',
        default_value='phase_1',
        description='Current phase name (e.g., phase_1, phase_2)'
    )

    # Nodes from original lite_localization.launch.py
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            {"yaml_filename": LaunchConfiguration('map_server_config_file')},
            {"topic_name": "map"},
            {"frame_id": "map"},
        ],
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    hdl_localization = Node(
        package='hdl_localization',
        executable='hdl_localization_composition',
        name='hdl_localization',
        parameters=[
            {"globalmap_pcd": LaunchConfiguration('globalmap_pcd')},
            {"convert_utm_to_local": True},
            {"odom_child_frame_id": "base_link"},
            {"use_imu": True},
            {"invert_acc": False},
            {"invert_gyro": False},
            {"cool_time_duration": 2.0},
            {"enable_robot_odometry_prediction": False},
            {"robot_odom_frame_id": "odom"},
            {"reg_method": "NDT_OMP"},
            {"ndt_neighbor_search_method": "DIRECT1"},
            {"ndt_neighbor_search_radius": 3.0},
            {"ndt_resolution": 1.5},
            {"downsample_resolution": 0.5},
            {"specify_init_pose": True},
            {"init_pos_x": 0.0},
            {"init_pos_y": 0.0},
            {"init_pos_z": 0.0},
            {"init_ori_w": 1.0},
            {"init_ori_x": 0.0},
            {"init_ori_y": 0.0},
            {"init_ori_z": 0.0},
            {"use_global_localization": False},
            {"t_diff": 0.25}
        ],
        remappings=[
            ('/velodyne_points', '/rslidar_points'),
            ('/gpsimu_driver/imu_data', '/imu/data'),
        ],
        output='screen'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dr_nav2_dir, 'launch', 'dr_nav2.launch.py')
        ),
        launch_arguments={
            'map_file': LaunchConfiguration('map_server_config_file')
        }.items()
    )

    # Waypoint Navigator Node
    waypoint_navigator = Node(
        package='nav_coffee',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        parameters=[{
            'phases_config_file': LaunchConfiguration('phases_config_file'),
            'current_phase': LaunchConfiguration('current_phase'),
        }],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(map_server_config_file_arg)
    ld.add_action(globalmap_pcd_arg)
    ld.add_action(phases_config_file_arg)
    ld.add_action(current_phase_arg)
    
    ld.add_action(map_server)
    ld.add_action(lifecycle_manager_map)
    ld.add_action(hdl_localization)
    ld.add_action(nav2_launch)
    ld.add_action(waypoint_navigator)

    return ld

