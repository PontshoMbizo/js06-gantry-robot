from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_dir = get_package_share_directory('main_gantry_assembly')

    # gz-sim's SystemLoader only searches GZ_SIM_SYSTEM_PLUGIN_PATH (plus a
    # couple of built-in defaults) when resolving a <plugin filename="..."/>
    # -- it does not fall back to LD_LIBRARY_PATH. gz_ros2_control's actual
    # shared library (libgz_ros2_control-system.so) lives in /opt/ros/jazzy/
    # lib, which isn't on that search path by default, so the plugin load
    # silently fails ("Could not find shared library") without this.
    existing_gz_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    gz_plugin_path = '/opt/ros/jazzy/lib'
    if existing_gz_plugin_path:
        gz_plugin_path += ':' + existing_gz_plugin_path

    # The URDF->SDF conversion rewrites mesh URIs from package://<pkg>/... to
    # model://<pkg>/..., and the Gazebo GUI resolves model:// purely via
    # GZ_SIM_RESOURCE_PATH -- it does not consult ROS's ament_index the way
    # robot_state_publisher/RViz do. Without this, the GUI logs "Unable to
    # find file[model://main_gantry_assembly/meshes/...]" for every mesh and
    # the robot renders as an empty/invisible model (physics still runs
    # fine -- this is a GUI/rendering-only issue). share_parent_dir is the
    # 'share' directory itself, so GZ_SIM_RESOURCE_PATH + '/main_gantry_
    # assembly/meshes/X.stl' lines up with where colcon actually installs
    # the meshes.
    share_parent_dir = os.path.dirname(package_dir)
    existing_gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = share_parent_dir
    if existing_gz_resource_path:
        gz_resource_path += ':' + existing_gz_resource_path

    urdf_path = os.path.join(package_dir, 'urdf', 'main_assembly.urdf')
    controllers_yaml = os.path.join(package_dir, 'config', 'gantry_controllers.yaml')
    world_path = os.path.join(package_dir, 'worlds', 'gantry_world.sdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read().replace(
            '__GANTRY_CONTROLLERS_YAML_PATH__', controllers_yaml
        )

    use_rviz = LaunchConfiguration('use_rviz')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_gantry',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'main_gantry_assembly',
            '-z', '0.05',
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller'],
        output='screen',
    )

    gantry_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gantry_control'),
                'launch',
                'gantry_control.launch.py',
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Launch RViz alongside Gazebo'
        ),

        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', gz_plugin_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),

        gz_sim,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,

        # Spawn controllers only after the robot entity actually exists in
        # the sim, and chain joint_state_broadcaster -> velocity_controller so
        # controller_manager sees state before the velocity controller claims
        # the command interfaces.
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[velocity_controller_spawner],
            )
        ),

        gantry_control_launch,
        rviz,
    ])
