#include "../../include/motorcontrol.h"
#include "../../include/wscommunicator.h"
#include "positioncontrol.h"
#include "../ForwardKinematics/kinematics.h"
#include "../../include/display.h"

// WSCOMMUNICATOR CONFIGS
const char *SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;
char message[200];
WsCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);

// physical configs
const float WHEEL_DIAMETER = 0.086;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;
const float LEFT_GAIN = 0.5;
const float RIGHT_GAIN = 0.7;
const float MAX_STEP = 0.5;
const float MAX_LINEAR_VELOCITY = 0.5;
const long MIN_PWM_PERCENT = 0.34;
const unsigned long POSITION_CONTROL_INTERVAL = 250;

MotorControl motorController(WHEEL_CIRCUMFERENCE, LEFT_GAIN, RIGHT_GAIN, MAX_STEP, MAX_LINEAR_VELOCITY, MIN_PWM_PERCENT, POSITION_CONTROL_INTERVAL);

Display display;
IntervalTimer messageTimer(500);

const float TRACK_WIDTH = 0.18;
ForwardKinematics forwardKinematics(TRACK_WIDTH, POSITION_CONTROL_INTERVAL);

const double GOAL_THRESHOLD = 0.2;
const double PC_TRACK_WIDTH = 0.18;
const double MAX_ANGULAR_VELOCITY = 1.0;
const double K_POSITION = 1.5;
const double K_ORIENTATION = 2.5;

PositionControl positionControl(
    0, 0, GOAL_THRESHOLD, PC_TRACK_WIDTH, MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY, K_POSITION, K_ORIENTATION, POSITION_CONTROL_INTERVAL);

// Waypoints
const double WAYPOINTS[][2] = {
    {2, 2},
    {2, 3},
    {2, 4},
    {3, 4},
    {4, 4}};
const int NUM_WAYPOINTS = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);
int currentWaypoint = 0;

//
// Setup function
//
void setup()
{
   // Start serial communication
   Serial.begin(115200);

   // Initialize the network communicator
   wsCommunicator.setup();

   // Initialize motor control
   motorController.setup();

   forwardKinematics.setup();

   // Initialize display
   display.setup();

   // Display IP address on the OLED screen
   display.drawString(0, 0, "IP: " + WiFi.localIP().toString());
   display.drawString(1, 0, "Position Control");

   // Initialize robot's position and orientation
   positionControl.setup();

   reset();
}

void reset()
{
   // Set robot pose to initial waypoint
   forwardKinematics.setPose(WAYPOINTS[0][0], WAYPOINTS[0][1], 0);
   currentWaypoint = 0;
   positionControl.setGoal(WAYPOINTS[currentWaypoint][0], WAYPOINTS[currentWaypoint][1]);
}

//
// Loop function
//
void loop()
{
   wsCommunicator.loopStep();

   if (wsCommunicator.resetFlagIsSet())
   {
      reset();
      wsCommunicator.clearResetFlag();
   }

   display.loopStep();

   motorController.loopStep(wsCommunicator.isEnabled());

   float leftVelocity = motorController.getLeftVelocity();
   float rightVelocity = motorController.getRightVelocity();
   forwardKinematics.loopStep(leftVelocity, rightVelocity);

   Pose pose = forwardKinematics.getPose();
   bool shouldUpdateVelocities = positionControl.loopStep(pose, leftVelocity, rightVelocity);

   // Check if the robot reached the current waypoint
   if (positionControl.goalReached(pose))
   {
      currentWaypoint++;
      if (currentWaypoint < NUM_WAYPOINTS)
      {
         positionControl.setGoal(WAYPOINTS[currentWaypoint][0], WAYPOINTS[currentWaypoint][1]);
      }
   }

   if (shouldUpdateVelocities)
   {
      motorController.setTargetVelocity(leftVelocity, rightVelocity);
   }

   if (messageTimer)
   {
      Pose pose = forwardKinematics.getPose();
      // snprintf(message, sizeof(message), "x=%f y=%f theta=%f vl=%f vr=%f", pose.x, pose.y, pose.theta, leftVelocity, rightVelocity);
      wsCommunicator.sendText(message, strlen(message));
   }
}
