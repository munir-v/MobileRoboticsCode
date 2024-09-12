import ..communication
import ..led

class LedBlinker implements Communicator:

  led := Led
  state := Communicator.DISABLED

  constructor:
    led.off

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
    led.on

  disable:
    if state == Communicator.DISABLED: return
    print "Disabling"
    state = Communicator.DISABLED
    led.off

main:
  led-blinker := LedBlinker
  comm := WsCommunication led-blinker --heartbeat-ms=1000

// make the LED blink every 250 ms, but only when the LedBlinker object is enabled.
  while true:
    if led-blinker.is-enabled:
      led-blinker.led.toggle
      sleep --ms=250
    else:
      sleep --ms=1000
      
