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
    motors.set-speed-forward -0.5

  disable:
    if state == Communicator.DISABLED: return
    print "Disabling"
    state = Communicator.DISABLED
    led.off
    motors.stop


main:
  led-blinker := LedBlinker
  comm := WsCommunication led-blinker --heartbeat-ms=7_500