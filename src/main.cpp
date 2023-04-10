#include <Arduino.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>

#include <vector>

#include "robot.h"

using namespace websockets;

WebsocketsServer server;
std::vector< std::pair<int, WebsocketsClient> > wsClients;
int nextClientId = 1;

xrp::Robot robot;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("No WiFi Module");
    while (true);
  }

  Serial.println("Setting up Access Point...");
  bool result = WiFi.softAP("XRP-WPILib", "0123456789");
  if (result == true) {
    Serial.println("Ready");
  }
  else {
    Serial.println("Failed...");
  }

  // Set up the server to listen AND only respond to an appropriate URI
  server.listen(3300, "/wpilibws");

  Serial.print(server.available() ? "WS Server running and ready on " : "Server not running on ");
  Serial.println("XRP Robot");
  Serial.print("IP Address: ");
  Serial.print(WiFi.localIP());
  Serial.print(", port: ");
  Serial.println(3300);

}

void pollWsClients() {
  for (auto& clientPair : wsClients) {
    clientPair.second.poll();
  }
}

void onWsMessage(WebsocketsClient& client, WebsocketsMessage message) {
  if (message.isText()) {
    // Process the message
    StaticJsonDocument<512> jsonDoc;
    DeserializationError error = deserializeJson(jsonDoc, message.data());
    if (error) {
      Serial.println(error.f_str());
      return;
    }

    // Hand this off to our processor
    // DEMO
    if (jsonDoc.containsKey("type")) {
      if (jsonDoc["type"] == "PWM") {
        
        int channel = atoi((jsonDoc["device"]).as<WSString>().c_str());
        
        if (jsonDoc.containsKey("data")) {
          auto data = jsonDoc["data"];
          if (data.containsKey("<speed")) {
            double value = atof((data["<speed"]).as<WSString>().c_str());
            Serial.print("PWM(");
            Serial.print(channel);
            Serial.print(") - ");
            Serial.println(value);
            robot.setPwmValue(channel, value);
          }
        }
        // if (jsonDoc.containsKey("<speed")) {
        //   Serial.println("Speed Message");
        //   double value = atof((jsonDoc["<speed"]).as<WSString>().c_str());
        //   robot.setPwmValue(channel, value);
        // }
        // else if (jsonDoc.containsKey("<position")) {
        //   Serial.println("Position Message");
        // }
      }
    }
  }
}

void onWsEvent(WebsocketsClient& client, WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionClosed) {
    for (auto it = wsClients.begin(); it != wsClients.end(); it++) {
      if (it->first == client.getId()) {
        Serial.print("Removing Client ID ");
        Serial.println(client.getId());
        wsClients.erase(it);
        break;
      }
    }
  }
}

int count = 0;
void loop() {
  if (server.poll()) {
    auto client = server.accept();
    client.onMessage(onWsMessage);
    client.onEvent(onWsEvent);

    if (client.available()) {
      Serial.println("Client accepted...");
      // Hook up events
      wsClients.push_back(std::make_pair(nextClientId, client));
      client.setId(nextClientId);
      nextClientId++;

      
      Serial.println("Event Hookup complete");
    }
  }

  pollWsClients();
}