"""Launch file for the single inverted pendulum. GIVEN -- do not modify.

Args:
    controller:      pid | mpc
    ic_mode:         upright | near_upright | downward | random
    enable_foxglove: true | false

Starts sim_node, the selected controller node, robot_state_publisher (fed
the xacro-processed URDF), and (if enabled) foxglove_bridge.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('single_inverted')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    controller = LaunchConfiguration('controller').perform(context)
    ic_mode = LaunchConfiguration('ic_mode').perform(context)
    enable_foxglove = LaunchConfiguration('enable_foxglove').perform(context).lower() == 'true'

    xacro_file = os.path.join(pkg_share, 'urdf', 'single_pendulum.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    controller_exe = {
        'pid': 'controller_pid',
        'mpc': 'controller_mpc',
    }.get(controller)
    if controller_exe is None:
        raise ValueError(f"Unknown controller '{controller}' -- expected pid|mpc")

    nodes = [
        Node(
            package='single_inverted',
            executable='sim_node',
            name='sim_node',
            output='screen',
            parameters=[params_file, {'ic_mode': ic_mode}],
        ),
        Node(
            package='single_inverted',
            executable=controller_exe,
            name=controller_exe,
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            remappings=[('joint_states', '/single_inverted/joint_states')],
        ),
    ]

    if enable_foxglove:
        nodes.append(
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                output='screen',
                parameters=[{'port': 8765}],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='pid',
                               description='pid | mpc'),
        DeclareLaunchArgument('ic_mode', default_value='downward',
                               description='upright | near_upright | downward | random'),
        DeclareLaunchArgument('enable_foxglove', default_value='true',
                               description='true | false'),
        OpaqueFunction(function=launch_setup),
    ])
