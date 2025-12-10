import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_rviz():
    rviz_config = os.path.join(          # 找到配置文件的完整路径
        get_package_share_directory('faster_lio'), 'config', 'c16.rviz'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config]  # 加载命令行参数
    )
    return rviz

def launch_fasterlio():
    declare_fasterlio_config_file_cmd = DeclareLaunchArgument(
        'fasterlio_config_file',
        default_value=os.path.join(
            get_package_share_directory('faster_lio'),'config','c16.yaml'
        )
    )

    fasterlio = Node(
        package='faster_lio',
        executable='run_mapping_online',
        name='faster_lio_online',
        output='screen',
        parameters=[
            LaunchConfiguration('fasterlio_config_file')
        ],
        # prefix=['xterm -e gdb -ex run --args'],
    )
    return declare_fasterlio_config_file_cmd, fasterlio

def generate_launch_description():
    ld = LaunchDescription()
    declare_fasterlio_config_file_cmd, fasterlio = launch_fasterlio()
    ld.add_action(declare_fasterlio_config_file_cmd)
    ld.add_action(fasterlio)
    rviz = launch_rviz()
    ld.add_action(rviz)
    return ld

