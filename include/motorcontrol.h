#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/*
Using the DRV8835 motor driver from Pololu.
- The PWM, analog output pin of each motor sets the speed (0 is stopped, 255 is full speed)
- The DIR, digital output pin of each motor sets the direction (wire with HIGH as forward)
*/

const int LEFT_MOTOR_DIR_PIN = 8;
const int LEFT_MOTOR_PWM_PIN = 9;
const int RIGHT_MOTOR_DIR_PIN = 44;
const int RIGHT_MOTOR_PWM_PIN = 7;

// TODO: These are backwards
typedef enum { DIRECTION_BACKWARD = LOW, DIRECTION_FORWARD = HIGH } MotorDirection;

class MotorControl {
  long leftDutyCycle;
  long rightDutyCycle;

 public:
  MotorControl() : leftDutyCycle(0), rightDutyCycle(0) {}

  void setup() {
    pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);

    setDirection(DIRECTION_FORWARD);
    stop();
  }

  void loopStep(bool isEnabled) {
    if (!isEnabled) {
      stop();
    }
  }

  void stop() {
    analogWrite(LEFT_MOTOR_PWM_PIN, 0);
    analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
  }

  void setDirection(MotorDirection direction) {
    digitalWrite(LEFT_MOTOR_DIR_PIN, direction);
    digitalWrite(RIGHT_MOTOR_DIR_PIN, direction);
  }

  void setPWMPercent(long percent) {
    percent = constrain(percent, 0, 100);
    long dutyCycle = map(percent, 0, 100, 0, 255);

    leftDutyCycle = dutyCycle;
    rightDutyCycle = dutyCycle;

    analogWrite(LEFT_MOTOR_PWM_PIN, leftDutyCycle);
    analogWrite(RIGHT_MOTOR_PWM_PIN, rightDutyCycle);
  }
};

#endif
