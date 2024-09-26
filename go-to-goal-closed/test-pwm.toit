import display show Display 
import communication show Communicator WsCommunication
import ..led
import ..motors
import ..pinout

class LedBlinker implements Communicator:

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
  led-blinker := LedBlinker
  comm := WsCommunication led-blinker --heartbeat-ms=1000

  //find right wheel duty factor
  // duty_factor/float := 1.0
  // right-wheel := led-blinker.motors.right-encoder.get-rotation-rate 50

  // led-blinker.motors.right-motor.set-pwm-duty-factor duty-factor
  // sleep --ms=1000
  
  // while duty_factor > 0:
  //   sleep --ms=100
  //   right-wheel-change := led-blinker.motors.right-encoder.get-rotation-rate 100
  //   if (right-wheel-change - right-wheel).abs > 0.001:
  //     duty-factor -= 0.01
  //     led-blinker.motors.right-motor.set-pwm-duty-factor duty-factor
  //     print duty-factor
  //   else:
  //     print "Breaking..."
  //     print duty-factor
  //     led-blinker.motors.right-motor.set-pwm-duty-factor 0.0 // Stop the motor
  //     break
      
  //find left wheel duty factor
  // left-wheel := led-blinker.motors.left-encoder.get-rotation-rate 50

  // led-blinker.motors.left-motor.set-pwm-duty-factor duty-factor
  // sleep --ms=1000
  
  // while duty_factor > 0:
  //   sleep --ms=100
  //   left-wheel-change := led-blinker.motors.left-encoder.get-rotation-rate 100
  //   if (left-wheel-change - left-wheel).abs > 0.001:
  //     duty-factor -= 0.01
  //     led-blinker.motors.left-motor.set-pwm-duty-factor duty-factor
  //     print duty-factor
  //   else:
  //     print "Breaking..."
  //     print duty-factor
  //     led-blinker.motors.left-motor.set-pwm-duty-factor 0.0 // Stop the motor
  //     break


  // find max speed
  // right_duty_factor/float := 1.0
  // led-blinker.motors.right-motor.set-pwm-duty-factor right_duty_factor
  
  // right-wheel := led-blinker.motors.right-encoder.get-rotation-rate 1000
  
  
  // left_duty_factor/float := 7.0
  // led-blinker.motors.left-motor.set-pwm-duty-factor left_duty_factor
  
  // sleep --ms=1000
  
  // right-wheel2 := led-blinker.motors.right-encoder.get-rotation-rate 1000
  // print right-wheel2 * 2.5 * 3.1459