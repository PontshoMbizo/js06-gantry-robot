from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    default_params = os.path.join(
        get_package_share_directory('gantry_control'),
        'config',
        'control_params.yaml',
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    control_node = Node(
        package='gantry_control',
        executable='control_node',
        name='gantry_control_node',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Path to the controller/motor/sensor coefficients YAML'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock (Gazebo sim time) instead of wall time'
        ),
        control_node,
    ])
