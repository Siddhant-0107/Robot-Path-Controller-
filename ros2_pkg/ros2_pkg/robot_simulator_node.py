"""
Robot simulator node: Simulates a differential-drive robot.
- Subscribes to /cmd_vel (Twist)
- Publishes /odom (Odometry)

This creates the feedback loop needed for the controller to work.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import time


class RobotSimulatorNode(Node):
    def __init__(self):
        super().__init__('robot_simulator')
        
        # Robot state (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Current commanded velocities
        self.v_cmd = 0.0
        self.omega_cmd = 0.0
        
        # Subscriber for velocity commands
        self.sub_cmd = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_cb, 10
        )
        
        # Publisher for odometry
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        
        # Simulation loop at 50Hz
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.simulation_step)
        
        self.last_time = time.time()
        self.start_time = time.time()
        
        self.get_logger().info('RobotSimulatorNode started - robot at (0, 0, 0)')

    def cmd_vel_cb(self, msg: Twist):
        self.v_cmd = msg.linear.x
        self.omega_cmd = msg.angular.z

    def simulation_step(self):
        # Simple differential drive kinematics
        # x_dot = v * cos(theta)
        # y_dot = v * sin(theta)
        # theta_dot = omega
        
        self.x += self.v_cmd * math.cos(self.theta) * self.dt
        self.y += self.v_cmd * math.sin(self.theta) * self.dt
        self.theta += self.omega_cmd * self.dt
        
        # Normalize theta to [-pi, pi]
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion (rotation around Z axis)
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.theta)
        
        # Also publish velocities
        odom.twist.twist.linear.x = self.v_cmd
        odom.twist.twist.angular.z = self.omega_cmd
        
        self.pub_odom.publish(odom)
        
        # Log progress every 2 seconds
        elapsed = time.time() - self.start_time
        if int(elapsed) % 2 == 0 and abs(elapsed - int(elapsed)) < self.dt:
            self.get_logger().info(
                f'Robot at ({self.x:.2f}, {self.y:.2f}), theta={math.degrees(self.theta):.1f}°, '
                f'v={self.v_cmd:.2f}, omega={self.omega_cmd:.2f}'
            )

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = RobotSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
