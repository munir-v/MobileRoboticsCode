#include "../../include/motorcontrol.h"
#include "../../include/wscommunicator.h"
#include "positioncontrol.h"
#include "../ForwardKinematics/kinematics.h"
#include "../../include/display.h"

// physical configs
const float WHEEL_DIAMETER = 0.086;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;
const float WHEEL_BASE = 0.2;

// Network configuration
const char *SSID = "Pomona";
const uint16_t PORT = 8181;
char message[100];

IntervalTimer messageTimer(500);

// position control configs
const unsigned long HEARTBEAT_INTERVAL = 1000;
const double GOALX = 1;
const double GOALY = 1;
const double GOAL_THRESHOLD = .1;
const double TRACK_WIDTH = 0;
const double MAX_LINEAR_VELOCITY = 0;
const double MAX_ANGULAR_VELOCITY = 0;
const double K_POSITION = 0;
const double K_ORIENTATION = 0;

// forward kinematics interval
const float FORWARD_KINEMATIC_INTERVAL = 100;

// // TODO THIS MIGHT NEED TO BE CHANGED
// double v_L, v_R = 0;

// Instances of classes
WsCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);
MotorControl motorController(0.2, .5, .5, .5, .5, 0.34, 250);
Display display;

const unsigned long POSITION_CONTROL_INTERVAL;
PositionControl positionControl(
    GOALX, GOALY, GOAL_THRESHOLD, TRACK_WIDTH, MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY, K_POSITION, K_ORIENTATION, POSITION_CONTROL_INTERVAL);

DifferentialDriveRobot diffDriveRobot(WHEEL_BASE); // Wheelbase of 0.2 meters

double totalTime = 0;
double delta_t = 0.25;
Pose curr_pose;
Pose goal_pose;

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

   // Set initial motor target velocity
   // motorController.setTargetVelocity(0.1); // 0.1 m/s

   // Initialize display
   display.setup();

   // Display IP address on the OLED screen
   display.drawString(0, 0, "IP: " + WiFi.localIP().toString());
   display.drawString(1, 0, "Position Control");

   // Initialize robot's position and orientation
   diffDriveRobot.setup(); // Reset x, y, and theta to zero
   positionControl.setup();

   totalTime = 0;
}
//
// Loop function
//
void loop()
{
   // Process WebSocket communication
   wsCommunicator.loopStep();

   if (wsCommunicator.resetFlagIsSet())
   {
      diffDriveRobot.reset();
      wsCommunicator.clearResetFlag();
   }

   display.loopStep();

   motorController.loopStep(wsCommunicator.isEnabled());

   float leftVelocity = motorController.getLeftVelocity();
   float rightVelocity = motorController.getRightVelocity();

   diffDriveRobot.forward_kinematics(leftVelocity, rightVelocity, FORWARD_KINEMATIC_INTERVAL);

   Pose pose = diffDriveRobot.getPose();
   bool updateVelocities = positionControl.loopStep(pose, leftVelocity, rightVelocity);

   if (updateVelocities)
   {
      motorController.setTargetVelocity(leftVelocity, rightVelocity);
   }

   // Display the current pose on the OLED screen every 100ms
   if (messageTimer)
   {
      Pose pose = diffDriveRobot.getPose();
      display.drawString(2, 0, "x: " + String(pose.x));
      display.drawString(3, 0, "y: " + String(pose.y));
      display.drawString(4, 0, "theta: " + String(pose.theta));
      snprintf(message, sizeof(message), "x=%f y =%f theta=%f vl=%f vr=%f", pose.x, pose.y, pose.theta, leftVelocity, rightVelocity);
      wsCommunicator.sendText(message, strlen(message));
   }
}