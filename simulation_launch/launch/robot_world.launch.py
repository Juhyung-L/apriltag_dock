import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('simulation_launch')
    launch_file_dir = os.path.join(pkg_share, 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file = LaunchConfiguration('world_file')
    robot_model_file = LaunchConfiguration('robot_model_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    declare_world_file = DeclareLaunchArgument(
        name='world_file',
        default_value='',
        description='Path to world file'
    )
    declare_robot_model_file = DeclareLaunchArgument(
        name='robot_model_file',
        default_value='',
        description='Path to robot model file'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
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

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'robot_model_file': robot_model_file,
            'use_sim_time': use_sim_time
        }.items()
    )
    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'robot_model_file': robot_model_file,
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    return LaunchDescription([
        declare_world_file,
        declare_robot_model_file,
        declare_use_sim_time,
        declare_x_pose,
        declare_y_pose,

        gzserver_launch,
        gzclient_launch,
        robot_state_publisher_launch,
        spawn_robot_launch
    ])