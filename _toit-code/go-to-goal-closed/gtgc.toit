import communication show Communicator WsCommunication
import ..led
import ..motors
import ..pinout
import ..utilities

class HeartbeatHandler implements Communicator:
  state := Communicator.DISABLED

  led/Led := Led
  motors/Motors := Motors

  constructor: halt

  halt:
    led.off
    motors.stop
    
  on-start address port:
  on-open:
  on-close: disable
  on-message message: enable

  is-enabled:
      return state == Communicator.ENABLED
  
  enable:
    if state == Communicator.ENABLED: return
    state = Communicator. ENABLED

  disable:
    if state == Communicator.DISABLED: return
    state = Communicator.DISABLED
    halt

class ProportionalControl:
  kp/float
  speed-factor/float := 0.0
  max-speed-step/float
  max-speed/float

  constructor .kp/float .max-speed-step/float .max-speed/float:

  compute-speed-factor desired-speed/float measured-speed/float:
    desired-speed = constrain desired-speed -max-speed max-speed
    error:= desired-speed - measured-speed
    u := constrain (kp * error) -max-speed-step max-speed-step
    speed-factor = constrain (speed-factor + u) -1.0 1.0
    return speed-factor

class MotorControl:
  static WHEEL-CIRCUMFERENCE ::= 7.86
  static LEFT-KP ::= 0.1
  static RIGHT-KP ::= 0.1
  static MAX-PEED-STEP ::= 0.1
  static MAX-SPEED ::= 0.9

  motors/Motors

  left-controller/ProportionalControl := ProportionalControl LEFT-KP MAX-PEED-STEP MAX-SPEED
  right-controller/ProportionalControl := ProportionalControl RIGHT-KP MAX-PEED-STEP MAX-SPEED

  left-time/int := 0
  right-time/int := 0

  constructor .motors/Motors:
    left-time = Time.monotonic-us
    right-time = Time.monotonic-us
  
  update-forward-speed desired-speed/float:
    now := Time.monotonic-us
    time-delta := (now - left-time).to-float / 1_000_000.0
    left-time = now
    right-time = now
    left-rot-per-s := motors.left-encoder.get-rotation-rate time-delta
    left-speed:= left-rot-per-s * WHEEL-CIRCUMFERENCE
    left-speed-factor:= left-controller.compute-speed-factor desired-speed left-speed
    motors.left-motor.set-speed-factor left-speed-factor


    now = Time.monotonic-us
    time-delta = (now - right-time).to-float / 1_000_000.0
    left-time = now
    right-time = now
    right-rot-per-s := motors.right-encoder.get-rotation-rate time-delta
    right-speed:= right-rot-per-s * WHEEL-CIRCUMFERENCE
    right-speed-factor := right-controller.compute-speed-factor desired-speed right-speed
    motors.right-motor.set-speed-factor right-speed-factor
    
    print left-speed 
    print right-speed

main:
  heartbeat-handler := HeartbeatHandler
  comm := WsCommunication heartbeat-handler --heartbeat-ms=1000

  motor-control:= MotorControl heartbeat-handler.motors
  motor-speed := 0.305

  duration-ms := 10_000
  control-update-ms := 100

  time-ms := 0

  while time-ms < duration-ms:
    motor-control.update-forward-speed motor-speed
    sleep --ms=control-update-ms
    time-ms += control-update-ms
  
  heartbeat-handler.motors.stop
  print "Done"
  