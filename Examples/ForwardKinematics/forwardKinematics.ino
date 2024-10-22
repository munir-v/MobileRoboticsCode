#include "../../include/motorcontrol.h"
#include "../../include/wscommunicator.h"
#include "forwardKinematics.h"

//
// Global State
//

// Network configuration
const char* SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;

// Instances of classes
WSCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);
Display display;
MotorControl motorController;
DifferentialDriveRobot diffDriveRobot(0.2);  // Wheelbase of 0.2 meters
unsigned long totalTime = 0;

//
// Setup function
//
void setup() {
  // Start serial communication
  Serial.begin(115200);

  // Initialize the network communicator
  wsCommunicator.setup();

  // Initialize motor control
  motorController.setup();

  // Set initial motor target velocity
  motorController.setTargetVelocity(0.1);  // 0.1 m/s

  // Initialize display
  display.setup();

  // Display IP address on the OLED screen
  display.drawString(0, 0, "IP: " + WiFi.localIP().toString());

  // Initialize robot's position and orientation
  diffDriveRobot.setup();  // Reset x, y, and theta to zero

  totalTime = 10;
}

//
// Loop function
//
void loop() {
  // Process WebSocket communication
  wsCommunicator.loopStep();

  // Check if the WebSocket communicator is enabled
  if (wsCommunicator.isEnabled()) {
    // Set motor PWM to 50%
    motorController.motorDriver.setPwmPercent(50);
    motorController.motorDriver.setLeftSpeed(20);
    motorController.motorDriver.setRightSpeed(25);

    // Get wheel velocities from the motor encoder
    double v_L = motorController.getLeftVelocity();  // Left wheel velocity
    double v_R = motorController.getRightVelocity(); // Right wheel velocity
    double t = 0.1;  // Time step (100ms)

    // Update robot's kinematics using wheel velocities and time step
    diffDriveRobot.forward_kinematics(v_L, v_R, t);
    totalTime -= t;
  } else {
    // Stop the motor if WebSocket is disabled
    motorController.motorDriver.stop();
  }

  if (totalTime <= 0) {
    // Stop the motor
    motorController.motorDriver.stop();
  }
}
