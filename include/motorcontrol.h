#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/*
We are using the DRV8835 breakout board from Pololu:
https://www.pololu.com/product/2135

For each motor
- The PWM, analog output pin of each motor sets the speed (0 is stopped, 255 is full speed)
- The DIR, digital output pin of each motor sets the direction
*/

const int LEFT_MOTOR_DIR_PIN = 8;
const int LEFT_MOTOR_PWM_PIN = 9;
const int RIGHT_MOTOR_DIR_PIN = 44;
const int RIGHT_MOTOR_PWM_PIN = 7;

typedef enum { DIRECTION_FORWARD = LOW, DIRECTION_BACKWARD = HIGH } MotorDirection;

class MotorControl {
 public:
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

  void setDirection(MotorDirection direction) { setDirection(direction, direction); }
  void setDirection(MotorDirection leftDirection, MotorDirection rightDirection) {
    digitalWrite(LEFT_MOTOR_DIR_PIN, leftDirection);
    digitalWrite(RIGHT_MOTOR_DIR_PIN, rightDirection);
  }

  void setPWMPercent(long percent) { setPWMPercent(percent, percent); }
  void setPWMPercent(long leftPercent, long rightPercent) {
    leftPercent = constrain(leftPercent, 0, 100);
    long leftDutyCycle = map(leftPercent, 0, 100, 0, 255);
    analogWrite(LEFT_MOTOR_PWM_PIN, leftDutyCycle);

    rightPercent = constrain(rightPercent, 0, 100);
    long rightDutyCycle = map(rightPercent, 0, 100, 0, 255);
    analogWrite(RIGHT_MOTOR_PWM_PIN, rightDutyCycle);
  }
};

#endif
