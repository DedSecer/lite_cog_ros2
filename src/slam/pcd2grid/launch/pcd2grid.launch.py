
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_my_octomap():
    return Node(
        package='octomap_server',
        executable='my_octomap',
        name='my_octomap',
        output='screen',
        parameters=[
            {'file_path': "/home/ysc/lite_cog_ros2/system/map/lite3.pcd"},
            {'filter_leaf_size_m': 0.05},

            {'frame_id': "map"},
            # {'base_frame_id': "base_footprint"},
            # {'use_height_map', 'False'},
            # {'colored_map', False},
            # {'color_factor', False},

            # {'point_cloud_min_x': },  ###default: -std::numeric_limits<double>::max()
            # {'point_cloud_max_x': },  ###default: std::numeric_limits<double>::max()
            # {'point_cloud_min_y': },  ###default: -std::numeric_limits<double>::max()
            # {'point_cloud_max_y': },  ###default: std::numeric_limits<double>::max()
            {'point_cloud_min_z': 0.4},  ###default: -100
            {'point_cloud_max_z': 1.2},  ###default: 100
            {'occupancy_min_z': 0.0},   ###default: -100
            {'occupancy_max_z': 1.6},   ###default: 100
            # {'min_x_size': },         ###default: 0.0
            # {'min_y_size': },         ###default: 0.0

            # {'filter_speckles': False},
            # {'filter_ground_plane': True},
            # {'ground_filter.distance': 0.10},
            # {'ground_filter.angle': 0.35},
            # {'ground_filter.plane_distance': 0.10},

            {'sensor_model.max_range': 100.0},
            {'resolution': 0.05},
            # {'sensor_model.hit': },
            # {'sensor_model.miss': },
            # {'sensor_model.min': },
            # {'sensor_model.max': },

            {'compress_map': True},
            # {'incremental_2D_projection': True},

            # {'max_depth: '}           ###default:
            # {'color.r: '}             ###default: 0.0
            # {'color.g: '}             ###default: 0.0
            # {'color.b: '}             ###default: 1.0
            # {'color.a: '}             ###default: 1.0
            # {'color_free.r: '}        ###default: 0.0
            # {'color_free.g: '}        ###default: 1.0
            # {'color_free.b: '}        ###default: 0.0
            # {'color_free.a: '}        ###default: 1.0 

            # {'publish_free_space': False},
            {'latch': True}

            # {'octomap_path: '}
        ],
        remappings=[
            ('/cloud_in', '/pcd'),
        ],
    )

def launch_map_saver_server():
    return Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        parameters=[
            {"save_map_timeout": 30000},
            {"free_thresh_default": 0.25},
            {"occupied_thresh_default": 0.65},
            {"map_subscribe_transient_local": True},
            {"filter_ground_plane": True}
        ],
    )

def launch_lifecycle_manager():
    return Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='nav2_lifecycle_manager',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['map_saver_server']}
        ]
    )

def launch_rviz():
    rviz_config = os.path.join(
        get_package_share_directory('pcd2grid'), 'rviz', 'config.rviz'
    )
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config]
    )

def generate_launch_description():
    ld = LaunchDescription()

    octomap_server = launch_my_octomap()
    ld.add_action(octomap_server)

    map_saver_server = launch_map_saver_server()
    ld.add_action(map_saver_server)

    lifecycle_manager = launch_lifecycle_manager()
    ld.add_action(lifecycle_manager)

    rviz = launch_rviz()
    ld.add_action(rviz)
    return ld
