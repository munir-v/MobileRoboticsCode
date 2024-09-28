#include "ledblinker.h"
#include "wsheartbeater.h"

// Network configuration
const char* SSID = "Pomona";
// const uint16_t PORT = 8181;
// const unsigned long HEARTBEAT_INTERVAL = 1000;

// LED Blinking configuration
const unsigned long BLINK_INTERVAL = 250;

// WSHeartbeater wsHeartbeater(PORT, HEARTBEAT_INTERVAL);
LedBlinker ledBlinker(BLINK_INTERVAL);

void setup() {
  Serial.begin(115200);
  wsHeartbeater.setup(SSID);
  ledBlinker.setup();
}

void loop() {
  wsHeartbeater.loopStep();
  ledBlinker.loopStep(wsHeartbeater.wsState == WS_ENABLED);
}
