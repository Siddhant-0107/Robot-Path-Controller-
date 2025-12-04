"""
Controller node: subscribes to `trajectory` (nav_msgs/Path) and `/odom`, computes velocity commands
and publishes `cmd_vel` (geometry_msgs/Twist). Implements a simple pure-pursuit style controller.

This node is intentionally minimal and educational.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import math


def pose_distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.declare_parameter('lookahead', 0.3)  # Reduced for tighter tracking
        self.declare_parameter('v_max', 0.5)
        self.declare_parameter('k_rot', 1.5)  # Reduced to prevent oscillation
        self.declare_parameter('goal_tolerance', 0.15)  # Stop when this close to goal

        self.lookahead = self.get_parameter('lookahead').value
        self.v_max = self.get_parameter('v_max').value
        self.k_rot = self.get_parameter('k_rot').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        self.trajectory = None
        self.odom = None
        self.current_index = 0  # Track progress along path
        self.goal_reached = False

        self.sub_traj = self.create_subscription(Path, 'trajectory', self.trajectory_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('ControllerNode started')

    def trajectory_cb(self, msg: Path):
        # store the incoming path (list of PoseStamped)
        self.trajectory = msg.poses
        self.current_index = 0  # Reset when new trajectory received
        self.goal_reached = False
        self.get_logger().info(f'Received trajectory with {len(self.trajectory)} poses')

    def odom_cb(self, msg: Odometry):
        self.odom = msg

    def control_loop(self):
        if self.trajectory is None or self.odom is None:
            return
        
        if self.goal_reached:
            # Already at goal, stop
            cmd = Twist()
            self.pub_cmd.publish(cmd)
            return

        # Current robot pose in map frame
        rx = self.odom.pose.pose.position.x
        ry = self.odom.pose.pose.position.y
        # yaw extraction from quaternion (simple)
        q = self.odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        rtheta = math.atan2(siny_cosp, cosy_cosp)

        # Check if we reached the goal
        goal = self.trajectory[-1]
        dist_to_goal = math.hypot(goal.pose.position.x - rx, goal.pose.position.y - ry)
        if dist_to_goal < self.goal_tolerance:
            self.goal_reached = True
            self.get_logger().info(f'Goal reached! Final position: ({rx:.2f}, {ry:.2f})')
            cmd = Twist()
            self.pub_cmd.publish(cmd)
            return

        # Find the closest point on path starting from current_index (don't go backwards)
        min_dist = float('inf')
        closest_idx = self.current_index
        
        # Search from current position forward (with some lookback for robustness)
        search_start = max(0, self.current_index - 5)
        for i in range(search_start, len(self.trajectory)):
            ps = self.trajectory[i]
            dist = math.hypot(ps.pose.position.x - rx, ps.pose.position.y - ry)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Update current index (only move forward, never backwards significantly)
        if closest_idx >= self.current_index:
            self.current_index = closest_idx
        
        # Find lookahead point: look ahead from closest point
        target_idx = self.current_index
        for i in range(self.current_index, len(self.trajectory)):
            ps = self.trajectory[i]
            dist = math.hypot(ps.pose.position.x - rx, ps.pose.position.y - ry)
            if dist >= self.lookahead:
                target_idx = i
                break
        else:
            # No point far enough ahead, use last point
            target_idx = len(self.trajectory) - 1
        
        target = self.trajectory[target_idx]
        tx = target.pose.position.x
        ty = target.pose.position.y

        # Compute control using a simple proportional heading controller + forward speed
        angle_to_target = math.atan2(ty - ry, tx - rx)
        angle_err = self._angle_diff(angle_to_target, rtheta)

        # Set forward speed proportional to cos of heading error (slow down when turning)
        # Also slow down when approaching goal
        speed_factor = max(0.2, math.cos(angle_err))  # Minimum 20% speed
        goal_factor = min(1.0, dist_to_goal / 1.0)  # Slow down within 1m of goal
        v = self.v_max * speed_factor * max(0.3, goal_factor)
        
        omega = self.k_rot * angle_err
        # Limit angular velocity to prevent spinning
        omega = max(-1.5, min(1.5, omega))

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.pub_cmd.publish(cmd)

    def _angle_diff(self, a, b):
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
