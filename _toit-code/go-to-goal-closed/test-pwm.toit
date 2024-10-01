import display show Display 
import communication show Communicator WsCommunication
import ..led
import ..motors
import ..pinout

class GoToGoal implements Communicator:

  led := Led
  state := Communicator.DISABLED
  display := Display SDA-PIN SCL-PIN
  motors := Motors

  constructor:
    led.off

  on-start address port: 
    print "$address:$port"
    display.add-text "$address"
    display.add-text --y=32 "$port"
    display.draw
  on-open: enable
  on-close: disable
  on-message message:
    print "Received message: $message"
    enable

  is-enabled:
    return state == Communicator.ENABLED

  enable:
    if state == Communicator.ENABLED: return
    print "Enabling"
    state = Communicator.ENABLED
    led.on

  disable:
    if state == Communicator.DISABLED: return
    print "Disabling"
    state = Communicator.DISABLED
    led.off
    motors.stop

main:
  go-to-goal := GoToGoal
  comm := WsCommunication go-to-goal --heartbeat-ms=1000

  // FIND RIGHT WHEEL DUTY FACTOR
  // duty_factor/float := 1.0
  // right-wheel := go-to-goal.motors.right-encoder.get-rotation-rate 50

  // go-to-goal.motors.right-motor.set-pwm-duty-factor duty-factor
  // sleep --ms=1000
  
  // while duty_factor > 0:
  //   sleep --ms=100
  //   right-wheel-change := go-to-goal.motors.right-encoder.get-rotation-rate 100
  //   if (right-wheel-change - right-wheel).abs > 0.001:
  //     duty-factor -= 0.01
  //     go-to-goal.motors.right-motor.set-pwm-duty-factor duty-factor
  //     print duty-factor
  //   else:
  //     print "Breaking..."
  //     print duty-factor
  //     go-to-goal.motors.right-motor.set-pwm-duty-factor 0.0 // Stop the motor
  //     break
      

  // FIND LEFT WHEEL DUTY FACTOR
  // left-wheel := go-to-goal.motors.left-encoder.get-rotation-rate 50

  // go-to-goal.motors.left-motor.set-pwm-duty-factor duty-factor
  // sleep --ms=1000
  
  // while duty_factor > 0:
  //   sleep --ms=100
  //   left-wheel-change := go-to-goal.motors.left-encoder.get-rotation-rate 100
  //   if (left-wheel-change - left-wheel).abs > 0.001:
  //     duty-factor -= 0.01
  //     go-to-goal.motors.left-motor.set-pwm-duty-factor duty-factor
  //     print duty-factor
  //   else:
  //     print "Breaking..."
  //     print duty-factor
  //     go-to-goal.motors.left-motor.set-pwm-duty-factor 0.0 // Stop the motor
  //     break


  // FIND MAX SPEED
  // right_duty_factor/float := 1.0
  // go-to-goal.motors.right-motor.set-pwm-duty-factor right_duty_factor
  // right-wheel := go-to-goal.motors.right-encoder.get-rotation-rate 1000
  
  // left_duty_factor/float := 7.0
  // go-to-goal.motors.left-motor.set-pwm-duty-factor left_duty_factor
  
  // sleep --ms=1000
  
  // right-wheel2 := go-to-goal.motors.right-encoder.get-rotation-rate 1000
  // print right-wheel2 * 2.5 * 3.1459


  // GO TO GOAL
  
  left_duty_factor/float := 0.5
  right_duty_factor/float := 0.5

  go-to-goal.motors.right-motor.set-pwm-duty-factor right-duty-factor
  go-to-goal.motors.left-motor.set-pwm-duty-factor left-duty-factor
  
  distance/float := 0.0

  while distance < 305.0: // (305cm = 10ft)
    sleep --ms=100
    right-wheel-change := go-to-goal.motors.right-encoder.get-rotation-rate 100
    left-wheel-change := go-to-goal.motors.left-encoder.get-rotation-rate 100

    right_wheel_distance := right-wheel-change * 2.5 * 3.1459
    left_wheel_distance := left-wheel-change * 2.5 * 3.1459

    distance = (right_wheel_distance + left_wheel_distance) / 2.0

    // update pwm using proportional control