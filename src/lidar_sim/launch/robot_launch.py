import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_dir = get_package_share_directory('lidar_sim')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'my_robot.urdf')).read_text()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt'),
        mode='realtime',
        gui=True
    )

    my_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'my_robot'},
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    # Add this Node to the LaunchDescription
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'lidar'],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('WEBOTS_USE_GPU', '1'),
        SetEnvironmentVariable('WEBOTS_OPENGL_PROFILE', 'core'),
        SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '0'),
        webots,
        my_robot_driver,
        static_transform_publisher,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])