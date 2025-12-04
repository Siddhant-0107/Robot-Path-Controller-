"""
Standalone ROS2-like simulation demo.

This script simulates what the ROS2 nodes would do:
- TrajectoryPublisher: generates and publishes a smooth trajectory
- ControllerNode: subscribes to trajectory and odom, publishes cmd_vel
- SimulatedRobot: integrates cmd_vel to produce odom

All running in a single process with message-passing via queues.
This demonstrates the ROS2 integration concept without requiring ROS2.
"""

import sys
import os
import time
import threading
import queue
from dataclasses import dataclass
from typing import List, Tuple

# Add src to path
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow

from smoothing import smooth_path
from trajectory import generate_trajectory


# --- Message types (simulating ROS2 messages) ---

@dataclass
class Path:
    """Simulates nav_msgs/Path"""
    poses: List[Tuple[float, float, float]]  # (x, y, timestamp)
    frame_id: str = "map"


@dataclass
class Odometry:
    """Simulates nav_msgs/Odometry"""
    x: float
    y: float
    theta: float
    vx: float = 0.0
    omega: float = 0.0


@dataclass
class Twist:
    """Simulates geometry_msgs/Twist"""
    linear_x: float
    angular_z: float


# --- Simulated ROS2 Nodes ---

class TrajectoryPublisher:
    """Simulates ros2_pkg/trajectory_publisher node"""
    
    def __init__(self, trajectory_queue: queue.Queue):
        self.trajectory_queue = trajectory_queue
        self.running = True
        
        # Generate smooth trajectory
        raw_waypoints = np.array([
            [0.0, 0.0],
            [1.0, 0.5],
            [2.0, 1.5],
            [3.0, 1.0],
            [4.0, 2.0],
            [5.0, 2.5],
        ])
        
        smooth = smooth_path(raw_waypoints, samples_per_segment=30)
        traj = generate_trajectory(smooth, n_samples=100, v_max=0.5, a_max=0.3)
        
        self.trajectory = Path(
            poses=[(traj['points'][i, 0], traj['points'][i, 1], traj['times'][i]) 
                   for i in range(len(traj['times']))],
            frame_id="map"
        )
        print(f"[TrajectoryPublisher] Generated trajectory with {len(self.trajectory.poses)} poses")
    
    def run(self):
        """Publish trajectory periodically"""
        while self.running:
            self.trajectory_queue.put(self.trajectory)
            time.sleep(1.0)
    
    def stop(self):
        self.running = False


class ControllerNode:
    """Simulates ros2_pkg/controller_node"""
    
    def __init__(self, trajectory_queue: queue.Queue, odom_queue: queue.Queue, 
                 cmd_queue: queue.Queue):
        self.trajectory_queue = trajectory_queue
        self.odom_queue = odom_queue
        self.cmd_queue = cmd_queue
        self.running = True
        
        self.trajectory = None
        self.current_odom = None
        self.lookahead = 0.3
        self.v_max = 0.5
        self.k_rot = 2.0
        
        print("[ControllerNode] Started")
    
    def run(self):
        """Control loop at 20 Hz"""
        while self.running:
            # Get latest trajectory
            try:
                while True:
                    self.trajectory = self.trajectory_queue.get_nowait()
            except queue.Empty:
                pass
            
            # Get latest odom
            try:
                while True:
                    self.current_odom = self.odom_queue.get_nowait()
            except queue.Empty:
                pass
            
            if self.trajectory is None or self.current_odom is None:
                time.sleep(0.05)
                continue
            
            # Compute control
            cmd = self._compute_control()
            self.cmd_queue.put(cmd)
            
            time.sleep(0.05)
    
    def _compute_control(self) -> Twist:
        x, y, theta = self.current_odom.x, self.current_odom.y, self.current_odom.theta
        
        # Check if reached goal first
        goal_x = self.trajectory.poses[-1][0]
        goal_y = self.trajectory.poses[-1][1]
        dist_to_goal = np.hypot(goal_x - x, goal_y - y)
        
        if dist_to_goal < 0.15:
            return Twist(linear_x=0.0, angular_z=0.0)
        
        # Find closest point on path, then look ahead from there
        min_dist = float('inf')
        closest_idx = 0
        for i, (px, py, t) in enumerate(self.trajectory.poses):
            dist = np.hypot(px - x, py - y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Look ahead from closest point
        target_idx = closest_idx
        for i in range(closest_idx, len(self.trajectory.poses)):
            px, py, t = self.trajectory.poses[i]
            dist = np.hypot(px - x, py - y)
            if dist >= self.lookahead:
                target_idx = i
                break
        else:
            target_idx = len(self.trajectory.poses) - 1
        
        tx, ty = self.trajectory.poses[target_idx][0], self.trajectory.poses[target_idx][1]
        
        # Compute heading error
        angle_to_target = np.arctan2(ty - y, tx - x)
        angle_err = self._angle_diff(angle_to_target, theta)
        
        # Proportional control with minimum speed
        speed_factor = max(0.3, np.cos(angle_err))  # Minimum 30% speed
        v = self.v_max * speed_factor
        
        # Slow down near goal
        if dist_to_goal < 1.0:
            v *= max(0.3, dist_to_goal)
        
        omega = self.k_rot * angle_err
        # Limit angular velocity
        omega = np.clip(omega, -1.5, 1.5)
        
        return Twist(linear_x=v, angular_z=omega)
    
    def _angle_diff(self, a, b):
        d = a - b
        while d > np.pi:
            d -= 2 * np.pi
        while d < -np.pi:
            d += 2 * np.pi
        return d
    
    def stop(self):
        self.running = False


class SimulatedRobot:
    """Simulates a differential-drive robot (like Turtlebot3)"""
    
    def __init__(self, cmd_queue: queue.Queue, odom_queue: queue.Queue):
        self.cmd_queue = cmd_queue
        self.odom_queue = odom_queue
        self.running = True
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.omega = 0.0
        
        self.dt = 0.05
        self.history = []
        
        print("[SimulatedRobot] Started at (0, 0, 0)")
    
    def run(self):
        """Physics update at 20 Hz"""
        while self.running:
            # Get latest command
            try:
                while True:
                    cmd = self.cmd_queue.get_nowait()
                    self.v = cmd.linear_x
                    self.omega = cmd.angular_z
            except queue.Empty:
                pass
            
            # Integrate kinematics
            self.x += self.v * np.cos(self.theta) * self.dt
            self.y += self.v * np.sin(self.theta) * self.dt
            self.theta += self.omega * self.dt
            
            # Publish odom
            odom = Odometry(x=self.x, y=self.y, theta=self.theta, 
                           vx=self.v, omega=self.omega)
            self.odom_queue.put(odom)
            
            # Record history
            self.history.append((self.x, self.y, self.theta, self.v, self.omega))
            
            time.sleep(self.dt)
    
    def stop(self):
        self.running = False


def main():
    print("=" * 60)
    print("ROS2-like Simulation Demo")
    print("This demonstrates the ROS2 integration without requiring ROS2")
    print("=" * 60)
    print()
    
    # Create message queues (simulating ROS2 topics)
    trajectory_queue = queue.Queue()
    odom_queue = queue.Queue()
    cmd_queue = queue.Queue()
    
    # Create nodes
    traj_pub = TrajectoryPublisher(trajectory_queue)
    controller = ControllerNode(trajectory_queue, odom_queue, cmd_queue)
    robot = SimulatedRobot(cmd_queue, odom_queue)
    
    # Start threads
    threads = [
        threading.Thread(target=traj_pub.run, daemon=True),
        threading.Thread(target=controller.run, daemon=True),
        threading.Thread(target=robot.run, daemon=True),
    ]
    
    for t in threads:
        t.start()
    
    print("\n[Main] Nodes started. Running simulation...")
    print("[Main] Close the matplotlib window to stop.\n")
    
    # Visualization
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_title("ROS2-like Simulation: Trajectory Tracking")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    
    # Wait for trajectory
    time.sleep(0.5)
    
    # Plot trajectory
    if traj_pub.trajectory:
        traj_x = [p[0] for p in traj_pub.trajectory.poses]
        traj_y = [p[1] for p in traj_pub.trajectory.poses]
        ax.plot(traj_x, traj_y, 'b-', linewidth=2, label='Planned trajectory')
        ax.plot(traj_x[0], traj_y[0], 'go', markersize=10, label='Start')
        ax.plot(traj_x[-1], traj_y[-1], 'r*', markersize=15, label='Goal')
    
    robot_dot, = ax.plot([], [], 'ko', markersize=8)
    robot_trail, = ax.plot([], [], 'g-', linewidth=1, alpha=0.5, label='Robot path')
    heading_line, = ax.plot([], [], 'r-', linewidth=2)
    
    ax.legend(loc='upper left')
    
    # Animation loop
    trail_x, trail_y = [], []
    try:
        while True:
            if len(robot.history) > 0:
                x, y, theta, v, omega = robot.history[-1]
                
                trail_x.append(x)
                trail_y.append(y)
                
                robot_dot.set_data([x], [y])
                robot_trail.set_data(trail_x, trail_y)
                
                # Heading arrow
                hx = [x, x + 0.2 * np.cos(theta)]
                hy = [y, y + 0.2 * np.sin(theta)]
                heading_line.set_data(hx, hy)
                
                ax.set_xlim(min(traj_x) - 0.5, max(traj_x) + 0.5)
                ax.set_ylim(min(traj_y) - 0.5, max(traj_y) + 0.5)
                
                # Check if reached goal
                dist_to_goal = np.hypot(traj_x[-1] - x, traj_y[-1] - y)
                if dist_to_goal < 0.1 and v < 0.01:
                    print(f"\n[Main] Robot reached goal at ({x:.2f}, {y:.2f})!")
                    break
            
            plt.pause(0.05)
            
    except KeyboardInterrupt:
        pass
    finally:
        # Stop nodes
        traj_pub.stop()
        controller.stop()
        robot.stop()
        
        print("\n[Main] Simulation complete!")
        print(f"[Main] Robot traveled {len(robot.history)} timesteps")
        
        plt.ioff()
        plt.show()


if __name__ == '__main__':
    main()
