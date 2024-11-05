import math

def update_position(pose, v_left, v_right, dt, track_width):
    v = (v_left + v_right) / 2
    theta_dot = (v_right - v_left) / track_width

    pose.x += v * math.cos(pose.theta) * dt
    pose.y += v * math.sin(pose.theta) * dt
    pose.theta += theta_dot * dt