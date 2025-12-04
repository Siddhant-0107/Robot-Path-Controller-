import time

import matplotlib.pyplot as plt
import numpy as np

from controller import simple_tracking_control


class Simulator:
    def __init__(self, trajectory, controller=simple_tracking_control, dt=0.05, v_des=0.8):
        self.traj = trajectory
        self.controller = controller
        self.dt = dt
        self.v_des = v_des

    def run(self, x0=(0.0, 0.0, 0.0), show=True):
        # initial state
        x, y, theta = x0
        times = self.traj['times']
        points = self.traj['points']
        total_time = times[-1]
        history = []
        t = 0.0
        idx = 0

        if show:
            plt.ion()
            fig, ax = plt.subplots(figsize=(8, 5))
            ax.plot(points[:, 0], points[:, 1], 'k-', label='smoothed path')
            ax.scatter(points[0, 0], points[0, 1], c='g', label='start')
            ax.scatter(points[-1, 0], points[-1, 1], c='r', label='goal')
            robot_dot, = ax.plot([], [], 'bo', label='robot')
            robot_dir, = ax.plot([], [], 'b-')
            ax.axis('equal')
            ax.legend()

        while t <= total_time + 1.0:
            # find desired point at current time by interpolation
            if t <= times[-1]:
                px = np.interp(t, times, points[:, 0])
                py = np.interp(t, times, points[:, 1])
            else:
                px, py = points[-1]

            v, omega = self.controller((x, y, theta), (px, py), v_des=self.v_des)
            # integrate unicycle
            x += v * np.cos(theta) * self.dt
            y += v * np.sin(theta) * self.dt
            theta += omega * self.dt
            history.append((t, x, y, theta, v, omega, px, py))

            if show:
                # matplotlib expects sequence-like inputs
                robot_dot.set_data([x], [y])
                # draw a small line showing heading
                hx = [x, x + 0.3 * np.cos(theta)]
                hy = [y, y + 0.3 * np.sin(theta)]
                robot_dir.set_data(hx, hy)
                plt.pause(0.001)

            t += self.dt
            # tiny sleep to keep UI responsive
            time.sleep(0.001)

        if show:
            plt.ioff()
            plt.show()
        return np.array(history)
