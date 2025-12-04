import math

import numpy as np


def angle_diff(a, b):
    d = a - b
    d = (d + math.pi) % (2 * math.pi) - math.pi
    return d


def simple_tracking_control(state, target_point, v_des=0.8, k_rho=0.8, k_alpha=2.0, v_max=1.0, omega_max=2.0):
    """Very simple controller: proportional on distance and heading.

    state: (x,y,theta)
    target_point: (x_t, y_t)
    returns: (v, omega)
    """
    x, y, theta = state
    x_t, y_t = target_point
    dx = x_t - x
    dy = y_t - y
    rho = math.hypot(dx, dy)
    # angle to target in world frame
    ang = math.atan2(dy, dx)
    alpha = angle_diff(ang, theta)
    # proportional control
    v = k_rho * rho
    v = min(v, v_max, v_des)
    omega = k_alpha * alpha
    omega = max(min(omega, omega_max), -omega_max)
    return v, omega
