import numpy as np
import matplotlib.pyplot as plt

class Pose:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x  
        self.y = y  
        self.theta = theta

def forward_kinematics(pose, v_left, v_right, dt, track_width):
    v = (v_left + v_right) / 2.0
    theta_dot = (v_right - v_left) / track_width

    pose.x += v * np.cos(pose.theta) * dt
    pose.y += v * np.sin(pose.theta) * dt
    pose.theta += theta_dot * dt

    return pose

def position_control(pose, goal, K_POSITION, K_ORIENTATION,
                     MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY, track_width):
    dx = goal[0] - pose.x
    dy = goal[1] - pose.y
    d = np.hypot(dx, dy)

    if d < 0.01:
        return 0.0, 0.0

    v = K_POSITION * d
    v = min(v, MAX_LINEAR_VELOCITY)

    angle_to_goal = np.arctan2(dy, dx)
    theta_error = angle_to_goal - pose.theta
    theta_error = (theta_error + np.pi) % (2 * np.pi) - np.pi  # Normalize angle

    theta_dot = K_ORIENTATION * theta_error
    theta_dot = np.clip(theta_dot, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)

    v_left = v - theta_dot * track_width / 2.0
    v_right = v + theta_dot * track_width / 2.0

    return v_left, v_right

dt = 0.01
TIME_END = 10.0
TRACK_WIDTH = 0.5
MEAN = 0
STD_DEV = 2

parameter_sets = [
    {'GOAL': (1, 1), 'K_POSITION': 1.0, 'K_ORIENTATION': 2.0,
     'MAX_LINEAR_VELOCITY': 0.5, 'MAX_ANGULAR_VELOCITY': 1.0},
    {'GOAL': (1, 1), 'K_POSITION': 0.5, 'K_ORIENTATION': 2.0,
     'MAX_LINEAR_VELOCITY': 0.5, 'MAX_ANGULAR_VELOCITY': 1.0},
    {'GOAL': (1, 1), 'K_POSITION': 1.0, 'K_ORIENTATION': 1.0,
     'MAX_LINEAR_VELOCITY': 0.5, 'MAX_ANGULAR_VELOCITY': 1.0},
    {'GOAL': (1, 1), 'K_POSITION': 1.0, 'K_ORIENTATION': 2.0,
     'MAX_LINEAR_VELOCITY': 0.3, 'MAX_ANGULAR_VELOCITY': 1.0},
    {'GOAL': (1, 1), 'K_POSITION': 1.0, 'K_ORIENTATION': 2.0,
     'MAX_LINEAR_VELOCITY': 0.5, 'MAX_ANGULAR_VELOCITY': 0.5},
    {'GOAL': (2, 0), 'K_POSITION': 1.0, 'K_ORIENTATION': 2.0,
     'MAX_LINEAR_VELOCITY': 0.5, 'MAX_ANGULAR_VELOCITY': 1.0},
    {'GOAL': (0, 2), 'K_POSITION': 1.0, 'K_ORIENTATION': 2.0,
     'MAX_LINEAR_VELOCITY': 0.5, 'MAX_ANGULAR_VELOCITY': 1.0},
    {'GOAL': (-1, -1), 'K_POSITION': 1.0, 'K_ORIENTATION': 2.0,
     'MAX_LINEAR_VELOCITY': 0.5, 'MAX_ANGULAR_VELOCITY': 1.0},
    {'GOAL': (1, -1), 'K_POSITION': 1.0, 'K_ORIENTATION': 2.0,
     'MAX_LINEAR_VELOCITY': 0.5, 'MAX_ANGULAR_VELOCITY': 1.0},
    {'GOAL': (-1, 1), 'K_POSITION': 1.0, 'K_ORIENTATION': 2.0,
     'MAX_LINEAR_VELOCITY': 0.5, 'MAX_ANGULAR_VELOCITY': 1.0},
]

trajectories = []
labels = []

for idx, params in enumerate(parameter_sets):
    pose = Pose(0, 0, 0)
    v_left = 0.0
    v_right = 0.0
    t = 0.0
    xs = []
    ys = []

    while t < TIME_END:
        pose = forward_kinematics(pose, v_left, v_right, dt, TRACK_WIDTH)
        v_left, v_right = position_control(
            pose,
            params['GOAL'],
            params['K_POSITION'],
            params['K_ORIENTATION'],
            params['MAX_LINEAR_VELOCITY'],
            params['MAX_ANGULAR_VELOCITY'],
            TRACK_WIDTH
        )
        v_left += np.random.normal(MEAN, STD_DEV)
        v_right += np.random.normal(MEAN, STD_DEV)
        xs.append(pose.x)
        ys.append(pose.y)
        t += dt

        dx = params['GOAL'][0] - pose.x
        dy = params['GOAL'][1] - pose.y
        d = np.hypot(dx, dy)
        if d < 0.01:
            break

    trajectories.append((xs, ys))
    label = f"Run {idx+1}: Goal={params['GOAL']}, Kp={params['K_POSITION']}, " \
            f"Ko={params['K_ORIENTATION']}, Vmax={params['MAX_LINEAR_VELOCITY']}, " \
            f"Wmax={params['MAX_ANGULAR_VELOCITY']}"
    labels.append(label)

plt.figure(figsize=(12, 8))

for idx, (xs, ys) in enumerate(trajectories):
    plt.plot(xs, ys, label=labels[idx])

plt.legend(loc='upper left', bbox_to_anchor=(1, 1))
plt.xlabel('X position')
plt.ylabel('Y position')
plt.title('Robot trajectories for different parameter combinations')
plt.grid(True)
plt.axis('equal')
plt.tight_layout()
plt.show()
