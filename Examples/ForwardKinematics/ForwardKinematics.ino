#include "../../include/motorcontrol.h"
#include "../../include/wscommunicator.h"
#include "kinematics.h"
#include "../../include/display.h"

// Network configuration
const char *SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;

// Instances of classes
WsCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);
MotorControl motorController(0.2, .5, .5, .5, .5, 0.34, 250);
Display display;
DifferentialDriveRobot diffDriveRobot(0.2); // Wheelbase of 0.2 meters

double totalTime = 0;
double delta_t = 0.25;
double lastMotorUpdateTime = 0;

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
    motorController.setTargetVelocity(0.2, 0.25);

    // Get wheel velocities from the motor encoder
    double v_L = motorController.getLeftVelocity();  // Left wheel velocity
    double v_R = motorController.getRightVelocity(); // Right wheel velocity

    unsigned long now = millis();

    // Update robot's kinematics using wheel velocities and time step
    if (now - lastMotorUpdateTime > (delta_t * 1000)) // Convert seconds to milliseconds
    {
      lastMotorUpdateTime = now;
      diffDriveRobot.forward_kinematics(v_L, v_R, delta_t);
      totalTime += delta_t;

      // print x, y, and theta
      double x = diffDriveRobot.get_x();
      double y = diffDriveRobot.get_y();
      double theta = diffDriveRobot.get_theta();
      // Serial.print("x: ");
      // Serial.print(x);
      // Serial.print(", y: ");
      // Serial.print(y);
      // Serial.print(", theta: ");
      // Serial.println(theta);
    }
  }
  else
  {
    // Stop the motor if WebSocket is disabled
    motorController.stop();
  }
}
