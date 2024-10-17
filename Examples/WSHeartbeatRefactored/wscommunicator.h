#include <WebSocketsServer.h>
#include <WiFi.h>

//
// State of WebSocket heartbeats
//

typedef enum { HEARTBEAT_ENABLED, HEARTBEAT_DISABLED } WsHeartbeatState;

//
// Forward declaration of the WebSocket event callback
//

class WsCommunicator;
void wsEventCB(WsCommunicator& wsComm, uint8_t num, WStype_t type, uint8_t* payload, size_t length);

//
// WebSocket server configuration, communication, and state
//

class WsCommunicator {
  friend void wsEventCB(WsCommunicator& wsComm, uint8_t num, WStype_t type, uint8_t* payload, size_t length);

 private:
  unsigned long heartbeatInterval;
  unsigned long heartbeatLastTime;

  const char* ssid;
  uint16_t port;
  WebSocketsServer webSocket;
  WsHeartbeatState hbState;

 public:
  WsCommunicator(const char* ssid, uint16_t port, unsigned long interval)
      : heartbeatInterval(interval)
      , heartbeatLastTime(0)
      , ssid(ssid)
      , port(port)
      , webSocket(port)
      , hbState(HEARTBEAT_DISABLED) {}

  void setup() {
    //
    // Setup WiFi
    //

    WiFi.begin(ssid);
    Serial.printf("\n[COMMUNICATOR::SETUP] Connecting to '%s'...", ssid);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.print("done\n");

    IPAddress ip = WiFi.localIP();
    Serial.printf("[COMMUNICATOR::SETUP] IP address: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);

    //
    // Start WebSockets server
    //

    auto wrappedCB = [this](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
      wsEventCB(*this, num, type, payload, length);
    };

    webSocket.begin();
    webSocket.onEvent(wrappedCB);
    Serial.printf("[COMMUNICATOR::SETUP] WebSocket server at ws://%d.%d.%d.%d:%d\n", ip[0], ip[1], ip[2], ip[3], port);
  }

  void loopStep() {
    // Update for processing WebSockets
    webSocket.loop();

    // Update for handling heartbeat
    unsigned long now = millis();
    if (now - heartbeatLastTime > heartbeatInterval) {
      if (hbState == HEARTBEAT_ENABLED) {
        hbState = HEARTBEAT_DISABLED;
        Serial.println("[COMMUNICATOR::HEARTBEAT] Heartbeat timeout");
      }
    }
  }

  bool isEnabled() { return hbState == HEARTBEAT_ENABLED; }
};

//
// WebSocket event callback
//

void wsEventCB(WsCommunicator& wsComm, uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  IPAddress ip;

  switch (type) {
    case WStype_DISCONNECTED:
      wsComm.hbState = HEARTBEAT_DISABLED;
      Serial.printf("[COMMUNICATOR::%u] Disconnected!\n", num);
      break;

    case WStype_CONNECTED:
      ip = wsComm.webSocket.remoteIP(num);
      Serial.printf("[COMMUNICATOR::%u] Client connected: %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      wsComm.webSocket.sendTXT(num, "Connected");
      break;

    case WStype_TEXT:
      if (strncmp((char*)payload, "heartbeat", length) == 0) {
        wsComm.hbState = HEARTBEAT_ENABLED;
        wsComm.heartbeatLastTime = millis();
      } else {
        Serial.printf("[COMMUNICATOR::%u] Received: %s\n", num, payload);
      }
      break;

    case WStype_BIN:
      Serial.printf("[COMMUNICATOR::%u] Received %u bytes of data\n", num, length);
      break;

    case WStype_ERROR:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT_FIN:
    case WStype_PING:
    case WStype_PONG:
      Serial.printf("[COMMUNICATOR::%u] Received unhandled event: %d\n", num, type);
      break;
  }
}
