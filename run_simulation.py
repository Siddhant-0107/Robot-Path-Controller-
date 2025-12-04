"""
Complete standalone trajectory tracking simulation with visualization.
No ROS2 required - runs entirely on Windows with matplotlib.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from smoothing import smooth_path
from trajectory import generate_trajectory


def compute_control(robot_x, robot_y, robot_theta, traj_points, current_idx, 
                    lookahead=0.3, v_max=0.5, k_rot=1.5):
    """Pure pursuit controller."""
    goal_x, goal_y = traj_points[-1]
    dist_to_goal = np.hypot(goal_x - robot_x, goal_y - robot_y)
    
    if dist_to_goal < 0.1:
        return 0.0, 0.0, current_idx, True  # Reached goal
    
    # Find closest point starting from current_idx
    min_dist = float('inf')
    closest_idx = current_idx
    for i in range(max(0, current_idx - 5), len(traj_points)):
        px, py = traj_points[i]
        dist = np.hypot(px - robot_x, py - robot_y)
        if dist < min_dist:
            min_dist = dist
            closest_idx = i
    
    # Only move forward
    if closest_idx > current_idx:
        current_idx = closest_idx
    
    # Find lookahead point
    target_idx = current_idx
    for i in range(current_idx, len(traj_points)):
        px, py = traj_points[i]
        dist = np.hypot(px - robot_x, py - robot_y)
        if dist >= lookahead:
            target_idx = i
            break
    else:
        target_idx = len(traj_points) - 1
    
    tx, ty = traj_points[target_idx]
    
    # Heading error
    angle_to_target = np.arctan2(ty - robot_y, tx - robot_x)
    angle_err = angle_to_target - robot_theta
    while angle_err > np.pi:
        angle_err -= 2 * np.pi
    while angle_err < -np.pi:
        angle_err += 2 * np.pi
    
    # Velocity control
    speed_factor = max(0.3, np.cos(angle_err))
    v = v_max * speed_factor
    if dist_to_goal < 1.0:
        v *= max(0.3, dist_to_goal)
    
    omega = k_rot * angle_err
    omega = np.clip(omega, -1.5, 1.5)
    
    return v, omega, current_idx, False


def main():
    print("=" * 60)
    print("Trajectory Tracking Simulation")
    print("=" * 60)
    
    # Generate trajectory
    waypoints = np.array([
        [0.0, 0.0],
        [1.0, 0.2],
        [2.0, -0.2],
        [3.5, 0.0],
        [5.0, 0.5],
        [6.0, 0.0],
    ])
    
    print("Generating smooth trajectory...")
    smooth = smooth_path(waypoints, samples_per_segment=30)
    traj = generate_trajectory(smooth, n_samples=100, v_max=0.5, a_max=0.3)
    points = traj['points']
    
    print(f"Trajectory: {len(points)} points")
    print(f"Start: ({points[0,0]:.2f}, {points[0,1]:.2f})")
    print(f"Goal:  ({points[-1,0]:.2f}, {points[-1,1]:.2f})")
    print()
    
    # Robot state
    robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0
    current_idx = 0
    dt = 0.05
    
    # History for plotting
    history_x, history_y = [robot_x], [robot_y]
    v_history, omega_history = [], []
    
    # Setup figure
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    fig.subplots_adjust(top=0.85)
    fig.suptitle('Trajectory Tracking Simulation', fontsize=14, y=0.98)
    
    # Left plot - path
    ax1.set_title('Robot Path')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    
    ax1.plot(points[:, 0], points[:, 1], 'b-', linewidth=2, label='Planned trajectory')
    ax1.plot(points[0, 0], points[0, 1], 'go', markersize=12, label='Start')
    ax1.plot(points[-1, 0], points[-1, 1], 'r*', markersize=15, label='Goal')
    robot_dot, = ax1.plot([robot_x], [robot_y], 'ko', markersize=10, label='Robot')
    robot_trail, = ax1.plot(history_x, history_y, 'g-', linewidth=2, alpha=0.7, label='Robot path')
    heading_line, = ax1.plot([robot_x, robot_x+0.3], [robot_y, robot_y], 'r-', linewidth=2, label='Heading')
    ax1.legend(loc='lower center', bbox_to_anchor=(0.5, 1.02), ncol=3, frameon=True)
    
    margin = 0.5
    ax1.set_xlim(min(points[:, 0]) - margin, max(points[:, 0]) + margin)
    ax1.set_ylim(min(points[:, 1]) - margin, max(points[:, 1]) + margin)
    
    # Right plot - velocities
    ax2.set_title('Velocity Commands')
    ax2.set_xlabel('Time step')
    ax2.set_ylabel('Velocity')
    ax2.grid(True, alpha=0.3)
    v_line, = ax2.plot([], [], 'b-', linewidth=2, label='Linear v (m/s)')
    omega_line, = ax2.plot([], [], 'r-', linewidth=2, label='Angular ω (rad/s)')
    ax2.legend(loc='lower center', bbox_to_anchor=(0.5, 1.02), ncol=2, frameon=True)
    ax2.set_ylim(-2, 2)
    ax2.set_xlim(0, 200)
    
    status_text = ax1.text(0.5, -0.1, '', transform=ax1.transAxes, ha='center', fontsize=10)
    
    goal_reached = False
    step = 0
    
    def update(frame):
        nonlocal robot_x, robot_y, robot_theta, current_idx, goal_reached, step
        
        if goal_reached:
            return robot_dot, robot_trail, heading_line, v_line, omega_line, status_text
        
        # Compute control
        v, omega, current_idx, goal_reached = compute_control(
            robot_x, robot_y, robot_theta, points, current_idx
        )
        
        # Update robot state
        robot_x += v * np.cos(robot_theta) * dt
        robot_y += v * np.sin(robot_theta) * dt
        robot_theta += omega * dt
        
        # Record history
        history_x.append(robot_x)
        history_y.append(robot_y)
        v_history.append(v)
        omega_history.append(omega)
        
        # Update plots
        robot_dot.set_data([robot_x], [robot_y])
        robot_trail.set_data(history_x, history_y)
        heading_line.set_data(
            [robot_x, robot_x + 0.3 * np.cos(robot_theta)],
            [robot_y, robot_y + 0.3 * np.sin(robot_theta)]
        )
        
        v_line.set_data(range(len(v_history)), v_history)
        omega_line.set_data(range(len(omega_history)), omega_history)
        ax2.set_xlim(0, max(200, len(v_history) + 10))
        
        dist_to_goal = np.hypot(points[-1, 0] - robot_x, points[-1, 1] - robot_y)
        status_text.set_text(f'Step: {step} | Pos: ({robot_x:.2f}, {robot_y:.2f}) | v={v:.2f} | dist_to_goal={dist_to_goal:.2f}')
        
        if goal_reached:
            ax1.set_title('Robot Path - GOAL REACHED! ✓', color='green', fontsize=12)
            print(f"\n*** GOAL REACHED! Final position: ({robot_x:.2f}, {robot_y:.2f}) ***")
        
        step += 1
        return robot_dot, robot_trail, heading_line, v_line, omega_line, status_text
    
    print("Starting animation... (close window to stop)")
    print("-" * 60)
    
    ani = FuncAnimation(fig, update, frames=500, interval=50, blit=False, repeat=False)
    plt.tight_layout()
    plt.show()
    
    print("-" * 60)
    print("Simulation complete!")


if __name__ == '__main__':
    main()
