#ifndef POSITIONCONTROL_H
#define POSITIONCONTROL_H

#include "../../include/forwardkinematics.h"
#include <math.h>
#include <algorithm>
#include <utility>

class PositionControl
{
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

    IntervalTimer updateTimer;

    PositionControl(double goalX, double goalY, double goalThreshold, double trackWidth,
                    double maxLinearVelocity, double maxAngularVelocity, double K_position, double K_orientation, unsigned long interval)
        : goalX(goalX), goalY(goalY), goalThreshold(goalThreshold), trackWidth(trackWidth), maxLinearVelocity(maxLinearVelocity), maxAngularVelocity(maxAngularVelocity), K_position(K_position), K_orientation(K_orientation), updateTimer(interval)
    {
    }

    void setup() {}

    bool loopStep(Pose pose, float &leftVelocity, float &rightVelocity)
    {
        if (!updateTimer)
        {
            return false;
        }

        float d = sqrt((goalX - pose.x) * (goalX - pose.x) + (goalY - pose.y) * (goalY - pose.y));
        if (d < goalThreshold)
        {
            leftVelocity = 0;
            rightVelocity = 0;
            return true;
        }

        float v = min(K_position * d, maxLinearVelocity);

        float angleToGoal = atan2(goalY - pose.y, goalX - pose.x);
        float angleError = angleToGoal - pose.theta;
        float thetaDot = min(K_orientation * angleError, maxAngularVelocity);

        leftVelocity = v - thetaDot * trackWidth / 2.0;
        rightVelocity = v + thetaDot * trackWidth / 2.0;

        return true;
    }

    void setGoal(float newGoalX, float newGoalY)
    {
        goalX = newGoalX;
        goalY = newGoalY;
    }
};

#endif // POSITIONCONTROL_H
