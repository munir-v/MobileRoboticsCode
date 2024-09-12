import ..motors

main:
  time-to-run := 3_000
  time-to-stop := 1_000
  time-between-updates := 500

  updates := time-to-run / time-between-updates

  half-speed := 0.5

  motors := Motors

  while true:

    print "Rotate in place..."
    motors.left-motor.set-speed 0.5
    motors.right-motor.set-speed -0.5

    updates.repeat:
      print "Left: $(%.2f (motors.left-encoder.get-speed*100)), Right: $(%.2f (motors.right-encoder.get-speed*100))"
      sleep --ms=time-between-updates

    print "Stopping..."
    motors.stop
    sleep --ms=time-to-stop

  motors.close
