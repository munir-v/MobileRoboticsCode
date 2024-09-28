#include <WebSocketsServer.h>
#include <WiFi.h>

typedef enum {
  WS_ENABLED,
  WS_DISABLED
} WsState;

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);

class WSHeartbeater {
 public:
  unsigned long heartbeatInterval;
  unsigned long heartbeatLastTime;
  uint16_t port;
  WebSocketsServer webSocket;
  WsState wsState;

  WSHeartbeater(uint16_t port, unsigned long interval) : heartbeatInterval(interval),
                                                         heartbeatLastTime(0),
                                                         port(port),
                                                         webSocket(port),
                                                         wsState(WS_DISABLED) {}

  void setup(const char* ssid) {
    // Setup WiFi
    WiFi.begin(ssid);
    Serial.printf("\n[SETUP] Connecting to '%s'... ", ssid);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.print("done\n");

    IPAddress ip = WiFi.localIP();
    Serial.printf("[SETUP] IP Address: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);

    // Start WebSockets server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.printf("[SETUP] WebSocketsServer started at ws://%d.%d.%d.%d:%d\n", ip[0], ip[1], ip[2], ip[3], port);
  }

  void loopStep() {
    // Update for processing WebSockets
    webSocket.loop();

    // Update for handling heartbeat
    unsigned long now = millis();
    if (now - heartbeatLastTime > heartbeatInterval) {
      if (wsState == WS_ENABLED) {
        wsState = WS_DISABLED;
        Serial.println("[HEARTBEAT] Heartbeat timeout");
      }
    }
  }
};

const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;
WSHeartbeater wsHeartbeater(PORT, HEARTBEAT_INTERVAL);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  IPAddress ip;

  switch (type) {
    case WStype_DISCONNECTED:
      wsHeartbeater.wsState = WS_DISABLED;
      Serial.printf("[%u] Disconnected!\n", num);
      break;

    case WStype_CONNECTED:
      ip = wsHeartbeater.webSocket.remoteIP(num);
      Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      wsHeartbeater.webSocket.sendTXT(num, "Connected");
      break;

    case WStype_TEXT:
      if (strcmp((char*)payload, "heartbeat") == 0) {
        wsHeartbeater.wsState = WS_ENABLED;
        wsHeartbeater.heartbeatLastTime = millis();
      } else {
        Serial.printf("[%u] get Text: %s\n", num, payload);
      }
      break;

    case WStype_BIN:
      Serial.printf("[%u] get binary length: %u\n", num, length);
      //   hexdump(payload, length);
      // webSocket.sendBIN(num, payload, length);
      break;

    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      Serial.printf("[%u] received unhandled WS event type: %d\n", num, type);
      break;
  }
}
