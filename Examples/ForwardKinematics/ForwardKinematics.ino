#include "../../include/motorcontrol.h"
#include "../../include/wscommunicator.h"
#include "kinematics.h"
#include "../../include/display.h"

// Network configuration
const char* SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;

// Instances of classes
WsCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);
Display display;
MotorControl motorController(0.2, .5, .5, .5, .5, 0.34, 100);
DifferentialDriveRobot diffDriveRobot(0.2);  // Wheelbase of 0.2 meters
unsigned long totalTime = 0;
unsigned long delta_t = 0.1;

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


  totalTime = 0;
}

//
// Loop function
//
void loop() {
  // Process WebSocket communication
  wsCommunicator.loopStep();

  // Check if the WebSocket communicator is enabled
  if (wsCommunicator.isEnabled()) {
    Serial.print("WebSocket is enabled\n");
    display.drawString(1, 0, "TEST");


    // Set left and right wheel velocities
    motorController.setTargetVelocity(1,2);

    // Get wheel velocities from the motor encoder
    double v_L = motorController.getLeftVelocity();  // Left wheel velocity
    double v_R = motorController.getRightVelocity(); // Right wheel velocity

    // Update robot's kinematics using wheel velocities and time step
    while (totalTime <= 10 ) {
      motorController.loopStep(true);
      diffDriveRobot.forward_kinematics(v_L, v_R, delta_t);
      v_L = motorController.getLeftVelocity();  // Left wheel velocity
      v_R = motorController.getRightVelocity(); // Right wheel velocity
      totalTime += delta_t;
    }
  } else {
    // Stop the motor if WebSocket is disabled
    motorController.stop();
  }
  motorController.stop();
}
