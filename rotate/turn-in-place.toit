import ..motors

main:
  motors := Motors

  while true:
    motors.left-motor.set-speed 0.5
    motors.right-motor.set-speed -0.5
    sleep --ms=500
