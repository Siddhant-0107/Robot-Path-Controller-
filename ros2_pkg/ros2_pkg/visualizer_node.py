"""
ROS2 Visualization Node - Shows trajectory tracking in matplotlib.

Subscribes to:
  - /trajectory (nav_msgs/Path) - the planned path
  - /odom (nav_msgs/Odometry) - robot position (simulated or real)
  - /cmd_vel (geometry_msgs/Twist) - velocity commands

This node provides a live matplotlib visualization of the robot following the trajectory.
"""
import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped


class SimulatedRobot:
    """Simulates robot motion based on cmd_vel commands."""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.dt = 0.05
    
    def update(self, v, omega):
        self.v = v
        self.omega = omega
        self.x += self.v * np.cos(self.theta) * self.dt
        self.y += self.v * np.sin(self.theta) * self.dt
        self.theta += self.omega * self.dt
        return self.x, self.y, self.theta


def compute_control(robot_x, robot_y, robot_theta, trajectory, lookahead=0.3, v_max=0.5, k_rot=1.5):
    """Compute velocity commands to follow trajectory."""
    if not trajectory:
        return 0.0, 0.0
    
    # Get goal
    goal_x = trajectory[-1].pose.position.x
    goal_y = trajectory[-1].pose.position.y
    dist_to_goal = np.hypot(goal_x - robot_x, goal_y - robot_y)
    
    # Check if reached goal
    if dist_to_goal < 0.15:
        return 0.0, 0.0
    
    # Find closest point on path
    min_dist = float('inf')
    closest_idx = 0
    for i, ps in enumerate(trajectory):
        px = ps.pose.position.x
        py = ps.pose.position.y
        dist = np.hypot(px - robot_x, py - robot_y)
        if dist < min_dist:
            min_dist = dist
            closest_idx = i
    
    # Find lookahead point
    target_idx = closest_idx
    for i in range(closest_idx, len(trajectory)):
        px = trajectory[i].pose.position.x
        py = trajectory[i].pose.position.y
        dist = np.hypot(px - robot_x, py - robot_y)
        if dist >= lookahead:
            target_idx = i
            break
    else:
        target_idx = len(trajectory) - 1
    
    tx = trajectory[target_idx].pose.position.x
    ty = trajectory[target_idx].pose.position.y
    
    # Compute heading error
    angle_to_target = np.arctan2(ty - robot_y, tx - robot_x)
    angle_err = angle_to_target - robot_theta
    # Normalize to [-pi, pi]
    while angle_err > np.pi:
        angle_err -= 2 * np.pi
    while angle_err < -np.pi:
        angle_err += 2 * np.pi
    
    # Proportional control
    speed_factor = max(0.3, np.cos(angle_err))
    v = v_max * speed_factor
    
    # Slow down near goal
    if dist_to_goal < 1.0:
        v *= max(0.3, dist_to_goal)
    
    omega = k_rot * angle_err
    omega = np.clip(omega, -1.5, 1.5)
    
    return v, omega


class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        
        # Subscribers
        self.traj_sub = self.create_subscription(Path, 'trajectory', self.trajectory_cb, 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        
        # Publisher for simulated odometry
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # State
        self.trajectory = None
        self.traj_x = []
        self.traj_y = []
        self.robot = SimulatedRobot()
        self.robot_history_x = []
        self.robot_history_y = []
        self.current_cmd = (0.0, 0.0)
        self.running = True
        
        # Timer for robot simulation
        self.timer = self.create_timer(0.05, self.simulation_step)
        
        self.get_logger().info('VisualizerNode started - waiting for trajectory...')
    
    def trajectory_cb(self, msg: Path):
        self.trajectory = msg.poses
        self.traj_x = [p.pose.position.x for p in msg.poses]
        self.traj_y = [p.pose.position.y for p in msg.poses]
        self.get_logger().info(f'Received trajectory with {len(self.trajectory)} poses')
    
    def cmd_cb(self, msg: Twist):
        self.current_cmd = (msg.linear.x, msg.angular.z)
    
    def simulation_step(self):
        """Update robot position and publish odometry."""
        # Compute control internally (don't rely on external controller)
        if self.trajectory:
            v, omega = compute_control(
                self.robot.x, self.robot.y, self.robot.theta, 
                self.trajectory
            )
            self.current_cmd = (v, omega)
        else:
            v, omega = 0.0, 0.0
            self.current_cmd = (0.0, 0.0)
        
        x, y, theta = self.robot.update(v, omega)
        
        self.robot_history_x.append(x)
        self.robot_history_y.append(y)
        
        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = np.sin(theta / 2)
        odom.pose.pose.orientation.w = np.cos(theta / 2)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)
    
    def get_state(self):
        """Return current state for visualization."""
        return {
            'traj_x': self.traj_x.copy(),
            'traj_y': self.traj_y.copy(),
            'robot_x': self.robot.x,
            'robot_y': self.robot.y,
            'robot_theta': self.robot.theta,
            'robot_v': self.robot.v,
            'robot_omega': self.robot.omega,
            'history_x': self.robot_history_x.copy(),
            'history_y': self.robot_history_y.copy(),
            'cmd': self.current_cmd,
        }


def visualization_loop(node):
    """Run matplotlib visualization in main thread."""
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 7))
    
    # Add extra space at top for legends
    fig.subplots_adjust(top=0.82)
    fig.suptitle('ROS2 Trajectory Tracking Visualization', fontsize=14, y=0.98)
    
    # Left plot - trajectory and robot
    ax1.set_title('Robot Path')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    
    traj_line, = ax1.plot([], [], 'b-', linewidth=2, label='Planned trajectory')
    robot_path, = ax1.plot([], [], 'g-', linewidth=1.5, alpha=0.7, label='Robot path')
    robot_dot, = ax1.plot([], [], 'ko', markersize=10, label='Robot')
    heading_line, = ax1.plot([], [], 'r-', linewidth=2, label='Heading')
    start_dot, = ax1.plot([], [], 'go', markersize=12, label='Start')
    goal_dot, = ax1.plot([], [], 'r*', markersize=15, label='Goal')
    # Place legend above the plot, outside the axes
    ax1.legend(loc='lower center', bbox_to_anchor=(0.5, 1.02), ncol=3, frameon=True, fancybox=True)
    
    # Right plot - velocity commands
    ax2.set_title('Velocity Commands')
    ax2.set_xlabel('Time step')
    ax2.set_ylabel('Velocity')
    ax2.grid(True, alpha=0.3)
    
    v_history = []
    omega_history = []
    v_line, = ax2.plot([], [], 'b-', label='Linear v (m/s)')
    omega_line, = ax2.plot([], [], 'r-', label='Angular ω (rad/s)')
    # Place legend above the plot, outside the axes
    ax2.legend(loc='lower center', bbox_to_anchor=(0.5, 1.02), ncol=2, frameon=True, fancybox=True)
    ax2.set_ylim(-2, 2)
    
    print("\n" + "="*60)
    print("ROS2 Visualization Running!")
    print("- Blue line: Planned trajectory")
    print("- Green line: Robot's actual path")
    print("- Black dot: Robot position")
    print("- Red line: Robot heading")
    print("Close the window to stop.")
    print("="*60 + "\n")
    
    step = 0
    try:
        while plt.fignum_exists(fig.number):
            state = node.get_state()
            
            # Update trajectory plot
            if state['traj_x']:
                traj_line.set_data(state['traj_x'], state['traj_y'])
                start_dot.set_data([state['traj_x'][0]], [state['traj_y'][0]])
                goal_dot.set_data([state['traj_x'][-1]], [state['traj_y'][-1]])
                
                # Auto-scale
                margin = 0.5
                ax1.set_xlim(min(state['traj_x']) - margin, max(state['traj_x']) + margin)
                ax1.set_ylim(min(state['traj_y']) - margin, max(state['traj_y']) + margin)
            
            # Update robot position
            robot_dot.set_data([state['robot_x']], [state['robot_y']])
            robot_path.set_data(state['history_x'], state['history_y'])
            
            # Heading arrow
            hx = [state['robot_x'], state['robot_x'] + 0.3 * np.cos(state['robot_theta'])]
            hy = [state['robot_y'], state['robot_y'] + 0.3 * np.sin(state['robot_theta'])]
            heading_line.set_data(hx, hy)
            
            # Update velocity plot
            v_history.append(state['cmd'][0])
            omega_history.append(state['cmd'][1])
            if len(v_history) > 200:
                v_history.pop(0)
                omega_history.pop(0)
            
            v_line.set_data(range(len(v_history)), v_history)
            omega_line.set_data(range(len(omega_history)), omega_history)
            ax2.set_xlim(0, max(200, len(v_history)))
            
            # Check if reached goal
            if state['traj_x']:
                dist_to_goal = np.hypot(state['traj_x'][-1] - state['robot_x'],
                                        state['traj_y'][-1] - state['robot_y'])
                if dist_to_goal < 0.15 and abs(state['robot_v']) < 0.05:
                    ax1.set_title(f'Robot Path - GOAL REACHED! ✓', color='green')
            
            plt.pause(0.05)
            step += 1
            
    except Exception as e:
        print(f"Visualization error: {e}")
    finally:
        plt.ioff()
        plt.close()


def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    
    # Run ROS2 spinning in a separate thread
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()
    
    # Run visualization in main thread (required for matplotlib)
    try:
        visualization_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
