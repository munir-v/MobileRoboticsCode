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
//  ▄    ▄          ▄                         ▄▄▄▄            ▀
//  ██  ██  ▄▄▄   ▄▄█▄▄   ▄▄▄    ▄ ▄▄         █   ▀▄  ▄ ▄▄  ▄▄▄    ▄   ▄   ▄▄▄    ▄ ▄▄
//  █ ██ █ █▀ ▀█    █    █▀ ▀█   █▀  ▀        █    █  █▀  ▀   █    ▀▄ ▄▀  █▀  █   █▀  ▀
//  █ ▀▀ █ █   █    █    █   █   █            █    █  █       █     █▄█   █▀▀▀▀   █
//  █    █ ▀█▄█▀    ▀▄▄  ▀█▄█▀   █            █▄▄▄▀   █     ▄▄█▄▄    █    ▀█▄▄▀   █

class MotorDriver {
  int dirPin;
  int pwmPin;

 public:
  MotorDriver(int dirPin, int pwmPin) : dirPin(dirPin), pwmPin(pwmPin) {}

  void setup() {
    pinMode(dirPin, OUTPUT);
    setDirection(DIRECTION_FORWARD);
    stop();
  }

  void stop() { analogWrite(pwmPin, 0); }

  void setDirection(MotorDirection direction) { digitalWrite(dirPin, direction); }

  void setPwmPercent(long percent) {
    percent = constrain(percent, 0, 100);
    long dutyCycle = map(percent, 0, 100, 0, 255);
    analogWrite(pwmPin, dutyCycle);
  }
};

class DualMotorDriver {
  MotorDriver leftDriver;
  MotorDriver rightDriver;

 public:
  DualMotorDriver()
      : leftDriver(LEFT_MOTOR_DIR_PIN, LEFT_MOTOR_PWM_PIN), rightDriver(RIGHT_MOTOR_DIR_PIN, RIGHT_MOTOR_PWM_PIN) {}

  void setup() {
    leftDriver.setup();
    rightDriver.setup();
  }

  void stop() {
    leftDriver.stop();
    rightDriver.stop();
  }

  void setDirection(MotorDirection direction) { setDirection(direction, direction); }
  void setDirection(MotorDirection leftDirection, MotorDirection rightDirection) {
    leftDriver.setDirection(leftDirection);
    rightDriver.setDirection(rightDirection);
  }

  void setPwmPercent(long percent) { setPwmPercent(percent, percent); }
  void setPwmPercent(long leftPercent, long rightPercent) {
    leftDriver.setPwmPercent(leftPercent);
    rightDriver.setPwmPercent(rightPercent);
  }
};

  }
};

#endif
