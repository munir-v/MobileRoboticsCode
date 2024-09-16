import gpio show Pin
import gpio.pwm show Pwm PwmChannel
import math show PI
import pulse_counter show Channel Unit

import ..pinout

class Motor:
  static FORWARD ::= 0
  static REVERSE ::= 1

  static FREQUENCY ::= 10_000

  dir-pin/Pin
  pwm-pin/Pin
  pwm-generator := Pwm --frequency=FREQUENCY
  pwm-channel/PwmChannel

  constructor dir/int pwm/int:
    dir-pin = Pin.out dir
    pwm-pin = Pin pwm
    pwm-channel = pwm-generator.start pwm-pin

  set-speed speed/float:
    // Clamp speed to [-1.0, 1.0]
    speed = speed > 1.0 ? 1.0 : (speed < -1.0 ? -1.0 : speed)
    direction := speed > 0.0 ? FORWARD : REVERSE
    dir-pin.set direction
    pwm-channel.set-duty-factor speed.abs

  stop:
    set-speed 0.0

  close:
    pwm-channel.close
    pwm-generator.close
    dir-pin.close
    pwm-pin.close


class Encoder:
  // 7 pulses per rotation, 2x due to quadrature, 50:1 gear ratio
  static COUNTS-PER-ROTATION := 7 * 2 * 50

  wheel-circumference/float

  pin-a/Pin
  pin-b/Pin
  unit := Unit // TODO: use filter?
  channel/Channel

  constructor a/int b/int wheel-diameter:

    wheel-circumference = wheel-diameter * PI

    pin-a = Pin.in a
    pin-b = Pin.in b

    channel = unit.add-channel
        pin-a
        --on-positive-edge=Unit.INCREMENT
        --on-negative-edge=Unit.DECREMENT
        --control-pin=pin-b
        --when-control-low=Unit.KEEP
        --when-control-high=Unit.REVERSE

  get-speed time-delta:
    count := unit.value
    unit.clear
    rotation := count.to-float / COUNTS-PER-ROTATION
    distance := rotation * wheel-circumference
    speed := distance / time-delta
    return speed

  close:
    channel.close
    unit.close
    pin-b.close
    pin-a.close

class Motors:
  left-motor := Motor LEFT-MOTOR-DIR-PIN LEFT-MOTOR-PWM-PIN
  right-motor := Motor RIGHT-MOTOR-DIR-PIN RIGHT-MOTOR-PWM-PIN

  left-encoder/Encoder
  right-encoder/Encoder

  constructor wheel-diameter:
    left-encoder = Encoder LEFT-ENCODER-PIN LEFT-ENCODER-CONTROL-PIN wheel-diameter
    right-encoder = Encoder RIGHT-ENCODER-PIN RIGHT-ENCODER-CONTROL-PIN wheel-diameter

  set-speed-forward speed/float:
    left-motor.set-speed speed
    right-motor.set-speed speed

  stop:
    left-motor.stop
    right-motor.stop

  close:
    left-motor.close
    right-motor.close

    left-encoder.close
    right-encoder.close
