from pose import Pose
import forwardkinematics as fk
import positioncontrol as pc

#  Change these constants to change the WMR Simulation settings
TRACK_WIDTH = 0
K_POSTION = 0
K_ORIENTATION = 0
MAX_LINEAR_VELOCITY = 0
MAX_ANGULAR_VELOCITY = 0
TIME_END = 0
DT = .5

def main():
    curr_pose = Pose()
    goal_pose = Pose()
    v_left = 0
    v_right = 0
    t = 0

    while t < TIME_END:
        curr_pose = fk.update_position(curr_pose, v_left, v_right, DT, TRACK_WIDTH)
        v_left, v_right = pc.update_wheel_velocities(curr_pose, goal_pose, K_POSTION, K_ORIENTATION, MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY, TRACK_WIDTH)
        t += DT

    print(curr_pose)


if __name__=="__main__":
    main()