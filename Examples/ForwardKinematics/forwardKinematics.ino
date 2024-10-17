#include "../../include/motorcontrol.h"
#include "../../include/wscommunicator.h"
#include "forwardKinematics.h"

//
// Global state
//

// Network configuration
const char* SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;
WSCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);

// source motor configs
MotorControl motorController;
DifferentialDriveRobot diffDriveRobot  = DifferentialDriveRobot(0.2)

// Setup
void setup() {
  Serial.begin(115200);
 
}

// Loop
void loop() {
  wsCommunicator.loopStep();

  if (wsCommunicator.isEnabled()) {
    motorController.motorDriver.setPwmPercent(50);

    double v_L = motorController.getLeftVelocity();   
    double v_R = motorController.getRightVelocity();  
    double t = 0.1;  // Time step of 100ms

    // Call forward kinematics with the motor speeds and time step
    diffDriveRobot.forward_kinematics(v_L, v_R, t);
  } else {
    motorDriver.stop();
  }
}
