#include "../../include/motorcontrol.h"
#include "../../include/wscommunicator.h"
#include "positioncontrol.h"
#include "../ForwardKinematics/kinematics.h"
#include "../../include/display.h"

// Network configuration
const char *SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;
const double GOALX = 1;
const double GOALY = 1;
const double GOAL_THRESHOLD = .1;
const double TRACK_WIDTH = 0;
const double MAX_LINEAR_VELOCITY = 0;
const double MAX_ANGULAR_VELOCITY = 0;
const double K_POSITION = 0;
const double K_ORIENTATION = 0;

// TODO THIS MIGHT NEED TO BE CHANGED
double v_L, v_R = 0;

// Instances of classes
WsCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);
MotorControl motorController(0.2, .5, .5, .5, .5, 0.34, 250);
Display display;
PositionControl positionControl(
    GOALX, GOALY, GOAL_THRESHOLD, TRACK_WIDTH, MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY, K_POSITION, K_ORIENTATION);
DifferentialDriveRobot diffDriveRobot(0.2); // Wheelbase of 0.2 meters

double totalTime = 0;
double delta_t = 0.25;
double lastMotorUpdateTime = 0;
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
    motorController.setTargetVelocity(0.1); // 0.1 m/s

    // Initialize display
    display.setup();

    // Display IP address on the OLED screen
    display.drawString(0, 0, "IP: " + WiFi.localIP().toString());

    // Initialize robot's position and orientation
    diffDriveRobot.setup(); // Reset x, y, and theta to zero

    totalTime = 0;
}

//
// Loop function
//
void loop()
{
    // Process WebSocket communication
    wsCommunicator.loopStep();
    motorController.loopStep(true);

    // Check if the WebSocket communicator is enabled
    if (wsCommunicator.isEnabled())
    {
        // Check if the total time is greater than 10 seconds
        if (totalTime > 10)
        {
            motorController.stop();
            return;
        }
        // Set left and right wheel velocities
        motorController.setTargetVelocity(v_L, v_R);

        unsigned long now = millis();

        // Update robot's kinematics using wheel velocities and time step
        if (now - lastMotorUpdateTime > (delta_t * 1000)) // Convert seconds to milliseconds
        {
            lastMotorUpdateTime = now;
            diffDriveRobot.forward_kinematics(v_L, v_R, delta_t);
            curr_pose.x = diffDriveRobot.get_x();
            curr_pose.y = diffDriveRobot.get_y();
            curr_pose.theta = diffDriveRobot.get_theta();
            v_L, v_R = positionControl.position_control(curr_pose, goal_pose);
            totalTime += delta_t;
        }
    }
    else
    {
        // Stop the motor if WebSocket is disabled
        motorController.stop();
    }
}
