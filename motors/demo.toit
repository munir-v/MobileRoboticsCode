import .motors

display-speed motors/Motors time-ms:
  time-delta := time-ms / 1000.0

  left-speed := (motors.left-encoder.get-speed time-delta) * 100
  right-speed := (motors.right-encoder.get-speed time-delta) * 100

  print "Left: $(%.2f left-speed) cm/s, Right: $(%.2f right-speed) cm/s"

main:
  time-to-run := 3_000
  time-to-stop := 1_000
  time-between-updates := 500

  updates := time-to-run / time-between-updates

  half-speed := 0.5

  motors := Motors 0.07

  while true:

    print "Running forward..."
    motors.set-speed-forward half-speed
    updates.repeat:
      display-speed motors time-between-updates
      sleep --ms=time-between-updates

    print "Stopping..."
    motors.stop
    sleep --ms=time-to-stop

    print "Running reverse..."
    motors.set-speed-forward -half-speed
    updates.repeat:
      display-speed motors time-between-updates
      sleep --ms=time-between-updates

    print "Stopping..."
    motors.stop
    sleep --ms=time-to-stop

  motors.close
