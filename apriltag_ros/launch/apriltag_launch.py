import os

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('apriltag_ros')
    params_file = os.path.join(pkg_share, 'params', 'apriltag_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    delcare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    apriltag_localization_node = Node(
        package='apriltag_ros',
        executable='main',
        name='apriltag_localization_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/image_rect', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info')
        ]
    )

    return LaunchDescription([
        delcare_use_sim_time,
        apriltag_localization_node
    ])