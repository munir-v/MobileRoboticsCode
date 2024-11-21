#!/usr/bin/env python
#!/opt/homebrew/Caskroom/miniforge/base/envs/visualize/bin/python
# You probably want to change the above line to: 

from __future__ import annotations

from collections import deque

from dataclasses import dataclass
from math import atan2, cos, pi, sin

import matplotlib
import matplotlib.pyplot as plt

matplotlib.use('TkAgg')


grid = [['0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '1', '1', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '1', '1', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '1', '1', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '1', '1', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '1', '1', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '0'], 
['0', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '0'], 
['0', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '0'], 
['0', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '0'], 
['0', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '0'], 
['0', '0', '0', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '0'], 
['0', '0', '0', '0', '0', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '0'], 
['0', '0', '0', '0', '0', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '0', '0', '0', '0', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '0'], 
['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '1', '1', '1', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '1', '1', '1', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '1', '1', '1', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '1', '1', '1', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'], 
['0', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '0']]

def wrap_to_pi(angle: float) -> float:
    return (angle + pi) % (2 * pi) - pi


@dataclass
class Pose:
    x: float = 0
    y: float = 0
    theta: float = 0

    def __str__(self) -> str:
        return f"{self.x:.3f} {self.y:.3f} {self.theta:.3f}"

    def copy(self) -> Pose:
        return Pose(self.x, self.y, self.theta)


class ForwardKinematics:
    def __init__(self, track_width: float):
        self.track_width = track_width
        self.pose = Pose()

    def update(self, left_velocity: float, right_velocity: float, dt: float) -> Pose:
        v = (right_velocity + left_velocity) / 2
        theta_dot = (right_velocity - left_velocity) / self.track_width

        self.pose.x += v * cos(self.pose.theta) * dt
        self.pose.y += v * sin(self.pose.theta) * dt
        self.pose.theta = wrap_to_pi(self.pose.theta + theta_dot * dt)

        return self.pose


class GoalPositionControl:
    def __init__(
        self,
        goal_x,
        goal_y,
        max_velocity_angular: float,
        max_velocity_linear: float,
        track_width: float,
        K_orientation: float,
        K_position: float,
    ):
        self.iteration = 0
        self.goal_x = goal_x
        self.goal_y = goal_y

        self.max_velocity_angular = max_velocity_angular
        self.max_velocity_linear = max_velocity_linear

        self.track_width = track_width

        self.K_orientation = K_orientation
        self.K_position = K_position

    def update(self, pose: Pose, threshold: float) -> tuple[float, float]:
        if self.iteration >= len(self.goal_x):
            return 0, 0  # Stop updating velocities if all goals are reached

        dx = self.goal_x[self.iteration] - pose.x
        dy = self.goal_y[self.iteration] - pose.y

        d_error = ((dx * dx) + (dy * dy)) ** 0.5

        if d_error < threshold:
            self.iteration += 1
            if self.iteration >= len(self.goal_x):  # Check after increment
                return 0, 0

        v = min(self.K_position * d_error, self.max_velocity_linear)

        theta_to_goal = atan2(dy, dx)
        theta_error = theta_to_goal - pose.theta
        theta_dot = min(self.K_orientation * theta_error, self.max_velocity_angular)
        theta_dot = max(theta_dot, -self.max_velocity_angular)

        left_velocity = v - theta_dot * self.track_width / 2
        right_velocity = v + theta_dot * self.track_width / 2

        return left_velocity, right_velocity

def bfs(r, c, tr, tc, grid):
    m, n = len(grid), len(grid[0])
    visited = set()
    parent = {(r, c): None}
    q = deque()
    visited.add((r,c))
    q.append((r,c))
    while q:
        current = q.popleft()
        if current[0] == tr and current[1] == tc:
            path = []
            while current is not None:
                path.append(current)
                current = parent[current]
            return path[::-1]
        row, col = current
        neighbors = [[row - 1, col], [row + 1, col], [row, col + 1], [row, col - 1], [row - 1, col - 1], [row - 1, col + 1], [row + 1, col + 1], [row + 1, col - 1]]
        for neighbor in neighbors:
            a, b = neighbor[0], neighbor[1]
            if (row in range(m) and col in range(n)) and grid[row][col] == "1" and (row, col) not in visited:
                q.append((a, b))
                visited.add((a, b))
                parent[(a, b)] = current

def simulate(duration: float,
             time_step: float,
             track_width: float,
             goal_x,
             goal_y,
             goal_threshold: float,
             max_velocity_angular: float,
             max_velocity_linear: float,
             K_orientation: float,
             K_position: float) -> list[Pose]:

    time = 0

    forward_kinematics = ForwardKinematics(track_width)
    position_control = GoalPositionControl( goal_x, goal_y, max_velocity_angular,
                                           max_velocity_linear, track_width,
        K_orientation, K_position
    )

    velocity_left = 0
    velocity_right = 0

    poses = [forward_kinematics.pose.copy()]

    # Numerical simulation loop
    while time < duration:
        pose = forward_kinematics.update(velocity_left, velocity_right, time_step)
        velocity_left, velocity_right = position_control.update(pose, goal_threshold)

        poses.append(pose.copy())

        time += time_step

    return poses


def main():
    # TODO: define an argument parser
    # TODO: program arguments

    start_x = 0
    start_y = 0
    goal_x = 10
    goal_y = 10

    print(bfs(start_x, start_y, goal_x, goal_y, grid))

    GOAL_X = [-1, 3, 2, -2, 0]
    GOAL_Y = [1, 1, 2, 3, -2]
    GOAL_THRESHOLD = 0.2

    MAX_VELOCITY_ANGULAR = 1
    MAX_VELOCITY_LINEAR = 0.3
    K_ORIENTATION = 1
    K_POSITION = 0.2

    TRACK_WIDTH = 0.17

    TIME_STEP = 0.1
    DURATION = 200

    poses = simulate(DURATION, TIME_STEP, TRACK_WIDTH, GOAL_X, GOAL_Y, GOAL_THRESHOLD,
             MAX_VELOCITY_ANGULAR, MAX_VELOCITY_LINEAR, K_ORIENTATION, K_POSITION)

    xs = [p.x for p in poses]
    ys = [p.y for p in poses]

    print(poses[-1])

    plt.plot(xs, ys)
    for i in range(len(GOAL_X)):
        plt.plot([GOAL_X[i]], [GOAL_Y[i]], 'o')
        plt.text(GOAL_X[i], GOAL_Y[i], f'{i+1}', fontsize=12)
        
    plt.gca().set_aspect('equal', adjustable='box')
    plt.get_current_fig_manager().window.wm_geometry('+2600+400')
    plt.show()

if __name__ == '__main__':
    main()
