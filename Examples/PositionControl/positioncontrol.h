#ifndef POSITIONCONTROL_H
#define POSITIONCONTROL_H

#include "../../include/forwardkinematics.h"
#include <math.h>
#include <algorithm>

class PositionControl
{
    // Private variables to store global state
private:


public:
    // Constructor initializes robot state
    double goalX;
    double goalY;
    double goalThreshold;
    double trackWidth;

    double maxLinearVelocity;
    double maxAngularVelocity;

    double K_position;
    double K_orientation;

    PositionControl( double goalX, double goalY, double goalThreshold, double trackWidth,
     double maxLinearVelocity, double maxAngularVelocity, double K_position, double K_orientation) 
     : goalX(goalX)
     , goalY(goalY)
     , goalThreshold(goalThreshold)
     , trackWidth(trackWidth)
     , maxLinearVelocity(maxLinearVelocity)
     , maxAngularVelocity(maxAngularVelocity)
     , K_position(K_position)
     , K_orientation(K_orientation)
     {}



    double position_control(Pose pose, Pose GOAL)
    {
        double d = distance(pose, GOAL);

        double v_left, v_right = 0;

        if (d < 0.1)
        {
            return v_left, v_right;
        }

        double v = K_position * d;
        v = std::min(v, maxLinearVelocity);

        double angle_to_goal = angle(pose, GOAL);
        double theta_error = angle_to_goal - pose.theta;
        double theta_dot = K_orientation * theta_error;
        theta_dot = std::min(theta_dot, maxAngularVelocity);

        v_left = v - theta_dot * TRACK_WIDTH / 2;
        v_right = v + theta_dot * TRACK_WIDTH / 2;

        return v_left, v_right;
    }

    double distance(Pose pose, Pose GOAL)
    {
        return sqrt(pow(GOAL.x - pose.x, 2) + pow(GOAL.y - pose.y, 2));
    }

    double angle(Pose pose, Pose GOAL)
    {
       return atan2(GOAL.y - pose.y, GOAL.x - pose.x);
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
