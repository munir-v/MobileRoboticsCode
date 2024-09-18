// import ..motors

// main:
//   time-to-run := 30_000
//   time-to-stop := 1_000
//   time-between-updates := 500

//   updates := time-to-run / time-between-updates

//   half-speed := 0.5

//   motors := Motors

//   while true:

//     print "Running forward..."
//     motors.set-speed-forward half-speed
//     updates.repeat:
//       print "Left: $(%.2f (motors.left-encoder.get-speed*100)), Right: $(%.2f (motors.right-encoder.get-speed*100))"
//       sleep --ms=time-between-updates

//     print "Stopping..."
//     motors.stop
//     sleep --ms=time-to-stop

//     print "Running reverse..."
//     motors.set-speed-forward -half-speed
//     updates.repeat:
//       print "Left: $(%.2f (motors.left-encoder.get-speed*100)), Right: $(%.2f (motors.right-encoder.get-speed*100))"
//       sleep --ms=time-between-updates

//     print "Stopping..."
//     motors.stop
//     sleep --ms=time-to-stop

//   motors.close

import ..display
import ..communication
import ..led
import ..motors

class LedBlinker implements Communicator:

  led := Led
  state := Communicator.DISABLED
  display := Display
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
  comm := WsCommunication led-blinker --heartbeat-ms=9_000