import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_name = 'slambot_description'
    this_pkg = 'slambot_gazebo'
    file_subpath = 'urdf/slambot.urdf.xacro'
    world_file_name='empty_world.sdf'
    
    world_path = os.path.join(
        get_package_share_directory(this_pkg),
        'worlds',
        world_file_name
    )

    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    # Not Sure is Static TF is needed for LIDAR
    static_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_lidar_tf',
        arguments=['0', '0', '0.175', '0', '0', '0', 'base_link', 'lidar_link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
 
    gazebo = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', world_path, 
            '--verbose', '-r'
            ],
        output='screen'
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.335' 
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    bridge_params = os.path.join(
        get_package_share_directory('slambot_gazebo'),
        'params',
        'slambot.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        start_gazebo_ros_bridge_cmd,
        node_robot_state_publisher,
        static_lidar_tf,
        spawn_entity,
        gazebo
    ])
