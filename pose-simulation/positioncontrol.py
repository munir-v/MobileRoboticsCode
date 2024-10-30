import math

def get_distance(pose1, pose2):
    return math.sqrt(((pose2.x - pose1.x)**2) +((pose2.y - pose1.y)**2))

def get_angle(pose1, pose2):
    # Fill this in
    return 0

def update_wheel_velocities(curr_pose, goal_pose, k_position, k_orientation, max_linear_velocity, max_angular_velocity, track_width):
    d = get_distance(goal_pose, curr_pose)

    if(d < 0.5):
        return (0, 0)
    else:
        v = k_position * d
        v = min(v, max_linear_velocity)

        angle_to_goal = get_angle(curr_pose, goal_pose)
        theta_error = angle_to_goal - curr_pose.theta
        theta_dot = k_orientation * theta_error
        theta_dot = min(theta_dot, max_angular_velocity)

        v_left = v - theta_dot * track_width / 2
        v_right = v + theta_dot * track_width / 2

        return (v_left, v_right)