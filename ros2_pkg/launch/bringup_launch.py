"""Python launch file to bring up the trajectory publisher and controller nodes."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    traj_pub = Node(
        package='ros2_pkg',
        executable='trajectory_publisher',
        name='trajectory_publisher',
        output='screen'
    )

    controller = Node(
        package='ros2_pkg',
        executable='controller_node',
        name='controller_node',
        output='screen'
    )

    ld.add_action(traj_pub)
    ld.add_action(controller)

    return ld
