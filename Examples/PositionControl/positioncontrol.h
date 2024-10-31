#ifndef POSITIONCONTROL_H
#define POSITIONCONTROL_H

#include "../../include/forwardkinematics.h"

class PositionControl
{
    // Private variables to store global state
private:


public:
    // Constructor initializes robot state
    PositionControl() {}



    double position_control(Pose pose, Pose GOAL)
    {
        double d = distance(pose, GOAL);

        double v_left, v_right = 0;

        if (d < 0.1)
        {
            return v_left, v_right;
        }

        double v = K_POSITION * d;
        v = min(v, MAX_LINEAR_VELOCITY);

        angle_to_goal = angle(pose, GOAL);
        theta_error = angle_to_goal - pose.theta;
        theta_dot = K_ORIENTATION * theta_error;
        theta_dot = min(theta_dot, MAX_ANGULAR_VELOCITY);

        v_left = v - θdot * TRACK_WIDTH / 2;
        v_right = v + θdot * TRACK_WIDTH / 2;

        return v_left, v_right;
    }

    double distance(Pose pose, Pose GOAL)
    {
        return sqrt(pow(GOAL.x - pose.x, 2) + pow(GOAL.y - pose.y, 2));
    }

    void setup()
    {
    }

    // Loop step function (placeholder for future use)
    void loopStep()
    {
    }
};


#endif // POSITIONCONTROL_H
