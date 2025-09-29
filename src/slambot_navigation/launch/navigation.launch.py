from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_nav = get_package_share_directory('slambot_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_gazebo = get_package_share_directory('slambot_gazebo')

    map_file = os.path.join(pkg_nav, 'maps', 'map.yaml')
    # param_file = os.path.join(pkg_nav, 'params', 'nav2_params.yaml')
    param_file = os.path.join(pkg_nav, 'params', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_nav, 'rviz', 'slambot_nav2.rviz')

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        )
    )

    odom_tf_node = Node(
        package='slambot_navigation',
        executable='odom_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': True}] 
    )

    # --- STATIC TF: map -> odom Not Sure if Needed---
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': param_file,
            'autostart': 'true'
        }.items()
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        use_sim_time,
        gazebo_launch,
        odom_tf_node,
        static_tf_node,
        nav2_bringup,
        rviz2_node
    ])
