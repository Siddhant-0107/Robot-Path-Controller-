"""
Debug simulation - prints robot position to verify movement.
"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
import matplotlib.pyplot as plt
from smoothing import smooth_path
from trajectory import generate_trajectory
from controller import simple_tracking_control

def main():
    # Waypoints
    waypoints = np.array([
        [0.0, 0.0],
        [2.0, 0.5],
        [4.0, 1.5],
        [6.0, 1.0],
        [8.0, 0.0],
    ])
    
    print("Generating trajectory...")
    smooth = smooth_path(waypoints, samples_per_segment=30)
    traj = generate_trajectory(smooth, n_samples=100, v_max=1.0, a_max=0.5)
    
    points = traj['points']
    times = traj['times']
    total_time = times[-1]
    
    print(f"Trajectory: {len(points)} points, total time: {total_time:.2f}s")
    print(f"Start: ({points[0,0]:.2f}, {points[0,1]:.2f})")
    print(f"Goal:  ({points[-1,0]:.2f}, {points[-1,1]:.2f})")
    print()
    
    # Robot state
    x, y, theta = 0.0, 0.0, 0.0
    dt = 0.05
    t = 0.0
    
    # Setup plot
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(points[:, 0], points[:, 1], 'b-', linewidth=2, label='Trajectory')
    ax.plot(points[0, 0], points[0, 1], 'go', markersize=12, label='Start')
    ax.plot(points[-1, 0], points[-1, 1], 'r*', markersize=15, label='Goal')
    robot_dot, = ax.plot([x], [y], 'ko', markersize=10)
    robot_trail_x, robot_trail_y = [x], [y]
    robot_trail, = ax.plot(robot_trail_x, robot_trail_y, 'g-', linewidth=1.5, alpha=0.7, label='Robot path')
    heading, = ax.plot([x, x+0.3], [y, y], 'r-', linewidth=2)
    
    ax.set_xlim(-1, 9)
    ax.set_ylim(-1, 3)
    ax.set_aspect('equal')
    ax.legend(loc='upper right')
    ax.set_title('Trajectory Tracking Simulation')
    ax.grid(True, alpha=0.3)
    
    step = 0
    print("Simulation running... (close window to stop)")
    print("-" * 50)
    
    while t <= total_time + 2.0:
        # Get target point at current time
        if t <= times[-1]:
            px = np.interp(t, times, points[:, 0])
            py = np.interp(t, times, points[:, 1])
        else:
            px, py = points[-1]
        
        # Control
        v, omega = simple_tracking_control((x, y, theta), (px, py), v_des=1.0)
        
        # Update robot state
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        theta += omega * dt
        
        # Print progress every 20 steps
        if step % 20 == 0:
            dist_to_goal = np.hypot(points[-1, 0] - x, points[-1, 1] - y)
            print(f"t={t:.1f}s | Robot: ({x:.2f}, {y:.2f}) | v={v:.2f} | dist_to_goal={dist_to_goal:.2f}")
        
        # Update plot
        robot_dot.set_data([x], [y])
        robot_trail_x.append(x)
        robot_trail_y.append(y)
        robot_trail.set_data(robot_trail_x, robot_trail_y)
        heading.set_data([x, x + 0.3 * np.cos(theta)], [y, y + 0.3 * np.sin(theta)])
        
        plt.pause(0.01)
        
        t += dt
        step += 1
        
        # Check if goal reached
        if np.hypot(points[-1, 0] - x, points[-1, 1] - y) < 0.1 and t > 1.0:
            print(f"\n*** GOAL REACHED at t={t:.1f}s! Final pos: ({x:.2f}, {y:.2f}) ***")
            break
    
    print("-" * 50)
    print("Simulation complete!")
    plt.ioff()
    plt.show()

if __name__ == '__main__':
    main()
