import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Config file path (use source directory for easy editing)
    source_config = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'src', 'racing', 'racing_control_2026', 'config', 'params.yaml'
    )
    # Fallback to installed config
    try:
        pkg_dir = get_package_share_directory('racing_control_2026')
        install_config = os.path.join(pkg_dir, 'config', 'params.yaml')
    except Exception:
        install_config = source_config

    config_file = source_config if os.path.exists(source_config) else install_config

    # Debug launch arguments
    start_phase_arg = DeclareLaunchArgument(
        'start_phase', default_value='1',
        description='Starting phase (1-4), skip earlier phases'
    )
    direction_arg = DeclareLaunchArgument(
        'direction', default_value='auto',
        description='Phase 3 route direction: auto/cw/ccw'
    )
    enable_obstacle_arg = DeclareLaunchArgument(
        'enable_obstacle', default_value='true',
        description='Enable YOLO obstacle avoidance'
    )
    enable_external_services_arg = DeclareLaunchArgument(
        'enable_external_services', default_value='true',
        description='Enable external services (QR scan, sign detection)'
    )

    # Racing controller node
    racing_controller_node = Node(
        package='racing_control_2026',
        executable='racing_controller',
        name='racing_controller',
        output='screen',
        parameters=[
            config_file,
            {
                'start_phase': LaunchConfiguration('start_phase'),
                'direction': LaunchConfiguration('direction'),
                'enable_obstacle': LaunchConfiguration('enable_obstacle'),
                'enable_external_services': LaunchConfiguration('enable_external_services'),
            }
        ],
        remappings=[
            ('/odom_combined', '/odom_combined'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    return LaunchDescription([
        start_phase_arg,
        direction_arg,
        enable_obstacle_arg,
        enable_external_services_arg,
        racing_controller_node,
    ])
