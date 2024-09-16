import ..communication
import ..led

class LedBlinker implements Communicator:

  led := Led
  state := Communicator.DISABLED
  blinking := false

  constructor:
    led.off
    task:: start-blinking

  on-start address port: print "$address:$port"
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
    blinking = true
    led.on

  disable:
    if state == Communicator.DISABLED: return
    print "Disabling"
    state = Communicator.DISABLED
    blinking = false
    led.off

  start-blinking:
    while true:
      if blinking:
        led.on
        sleep --ms=250
        led.off
        sleep --ms=250
      else:
        sleep --ms=1000

main:  
  led-blinker := LedBlinker  
  comm := WsCommunication led-blinker --heartbeat-ms=1000
