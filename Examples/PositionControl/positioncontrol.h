#ifndef POSITIONCONTROL_H
#define POSITIONCONTROL_H

#include "../../include/forwardkinematics.h"
#include <math.h>
#include <algorithm>
#include <utility>


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

   IntervalTimer updateTimer;

   PositionControl( double goalX, double goalY, double goalThreshold, double trackWidth,
    double maxLinearVelocity, double maxAngularVelocity, double K_position, double K_orientation, unsigned long interval)
    : goalX(goalX)
    , goalY(goalY)
    , goalThreshold(goalThreshold)
    , trackWidth(trackWidth)
    , maxLinearVelocity(maxLinearVelocity)
    , maxAngularVelocity(maxAngularVelocity)
    , K_position(K_position)
    , K_orientation(K_orientation)
    , updateTimer(interval)
    {}

   std::pair<double, double> position_control(Pose pose, Pose GOAL)
   {
       double d = distance(pose, GOAL);
       double v_left, v_right = 0;

       if (d < goalThreshold)
       {
           return std::make_pair(0, 0);
       }

       double v = K_position * d;
       v = std::min(v, maxLinearVelocity);

       double angle_to_goal = angle(pose, GOAL);
       double theta_error = angle_to_goal - pose.theta;
       double theta_dot = K_orientation * theta_error;
       theta_dot = std::min(theta_dot, maxAngularVelocity);

       v_left = v - theta_dot * trackWidth / 2.0;
       v_right = v + theta_dot * trackWidth / 2.0;

       return std::make_pair(v_left, v_right);
   }

   double distance(Pose pose, Pose GOAL)
   {
       return sqrt(pow(GOAL.x - pose.x, 2) + pow(GOAL.y - pose.y, 2));
   }


   double angle(Pose pose, Pose GOAL)
   {
      return atan2(GOAL.y - pose.y, GOAL.x - pose.x);
   }

   void setup() {}

   // Loop step returns a bool but as a side effect, also sets velocities using position control function
   bool loopStep(Pose pose, float& leftVelocity, float& rightVelocity){
       if (!updateTimer) {
           return false;
       }
       // target pose
       Pose targetPose = Pose(goalX, goalY, K_orientation);

       std::pair<double, double> velocities = position_control(pose, targetPose);
       leftVelocity = velocities.first;
       rightVelocity = velocities.second;

       return true;
       }
};

#endif // POSITIONCONTROL_H
