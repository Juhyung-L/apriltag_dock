import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('simulation_launch')
    
    rviz_config_file_path = os.path.join(pkg_share, 'config', 'urdf_config.rviz')
    ekf_config_file_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    world_file_path = os.path.join(pkg_share, 'worlds', 'apriltag_test.world')
    robot_model_file_path = os.path.join(pkg_share, 'urdf', 'mobile_bot.urdf')

    # launch configuration variables
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    world_file = LaunchConfiguration('world_file')
    robot_model_file = LaunchConfiguration('robot_model_file')

    # declare launch arguments
    delcare_rviz_config_file = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=rviz_config_file_path,
        description='Path to RViz config file'
    )
    delcare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_x_pose = DeclareLaunchArgument(
        name='x_pose',
        default_value='0.0',
        description='X-coordinate of robot spawn point'
    )
    declare_y_pose = DeclareLaunchArgument(
        name='y_pose',
        default_value='0.0',
        description='Y-coordinate of robot spawn point'
    )
    declare_world_file = DeclareLaunchArgument(
        name='world_file',
        default_value=world_file_path,
        description='Path to world file'
    )
    declare_robot_model_file = DeclareLaunchArgument(
        name='robot_model_file',
        default_value=robot_model_file_path,
        description='Path to robot model file'
    )

    # declare nodes to launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file_path, 
                    {'use_sim_time': use_sim_time}]
    )

    # launch other launch files
    robot_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot_world.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_sim_time': use_sim_time,
            'robot_model_file': robot_model_file,
            'world_file': world_file
        }.items()
    )

    return LaunchDescription([
        delcare_rviz_config_file,
        delcare_use_sim_time,
        declare_x_pose,
        declare_y_pose,
        declare_world_file,
        declare_robot_model_file,

        robot_world_launch,
        robot_localization_node,
        rviz_node
    ])
