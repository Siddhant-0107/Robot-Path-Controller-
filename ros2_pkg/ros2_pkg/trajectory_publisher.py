"""
ROS2 node that publishes a time-parameterized trajectory as a nav_msgs/Path.

This node uses the project's smoothing and trajectory generator from src/ so that it
publishes a truly smooth, time-parameterized path.
"""
import sys
import os

# Add the src folder to path so we can import smoothing and trajectory modules
_src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'src'))
if _src_path not in sys.path:
    sys.path.insert(0, _src_path)

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# Import project modules
from smoothing import smooth_path
from trajectory import generate_trajectory


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.pub = self.create_publisher(Path, 'trajectory', 10)

        # Parameters (could be exposed as ROS2 params)
        self.declare_parameter('v_max', 0.5)
        self.declare_parameter('a_max', 0.3)
        self.v_max = self.get_parameter('v_max').value
        self.a_max = self.get_parameter('a_max').value

        # Example coarse waypoints (a gentle S-shaped path)
        self.raw_waypoints = np.array([
            [0.0, 0.0],
            [1.0, 0.2],
            [2.0, -0.2],
            [3.5, 0.0],
            [5.0, 0.5],
            [6.0, 0.0],
        ])

        # Pre-compute smoothed trajectory
        smooth = smooth_path(self.raw_waypoints, samples_per_segment=30)
        self.traj = generate_trajectory(smooth, n_samples=100, v_max=self.v_max, a_max=self.a_max)

        # Publish once per second
        self.timer = self.create_timer(1.0, self.publish_trajectory)
        self.get_logger().info('TrajectoryPublisher started (smooth trajectory computed)')

    def publish_trajectory(self):
        path = Path()
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = 'map'
        path.header = hdr

        pts = self.traj['points']
        times = self.traj['times']

        for i in range(len(pts)):
            ps = PoseStamped()
            # Use trajectory time offset for stamp so downstream can interpolate
            sec = int(times[i])
            nanosec = int((times[i] - sec) * 1e9)
            ps.header.stamp.sec = hdr.stamp.sec + sec
            ps.header.stamp.nanosec = hdr.stamp.nanosec + nanosec
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(pts[i, 0])
            ps.pose.position.y = float(pts[i, 1])
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self.pub.publish(path)
        self.get_logger().info(f'Published smooth trajectory with {len(path.poses)} poses')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
