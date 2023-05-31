#include <Arduino.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>

#include <vector>

#include "robot.h"
#include "wpilibws_processor.h"

#define USE_AP true

using namespace websockets;


WebsocketsServer server;
std::vector< std::pair<int, WebsocketsClient> > wsClients;
int nextClientId = 1;

wpilibws::WPILibWSProcessor wsMsgProcessor;

xrp::Robot robot;

std::unordered_map<WSString, int> messageCounts;

void onDSEnabledMessage(bool enabled) {
  Serial.print("DS Enabled: ");
  Serial.println(enabled);
  robot.setEnabled(enabled);
}

void onPWMMessage(int channel, double value) {
  robot.setPwmValue(channel, value);
}

void onEncoderInitMessage(int channel, bool init, int chA, int chB) {
  Serial.print("Encoder(");
  Serial.print(channel);
  Serial.print(") - init? ");
  Serial.print(init);
  Serial.print(" - chA: ");
  Serial.print(chA);
  Serial.print(" - chB: ");
  Serial.println(chB);
  if (init) {
    robot.configureEncoder(channel, chA, chB);
  }
}

void onDIOMessage(int channel, bool value) {
  robot.setDioValue(channel, value);
}

void hookupWSMessageHandlers() {
  // Hook up the event listeners to the message processor
  wsMsgProcessor.onDSEnabledMessage(onDSEnabledMessage);
  wsMsgProcessor.onPWMMessage(onPWMMessage);
  wsMsgProcessor.onEncoderInitMessage(onEncoderInitMessage);
  wsMsgProcessor.onDIOMessage(onDIOMessage);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("No WiFi Module");
    while (true);
  }

  WiFi.setHostname("XRP-Bot");

  if (USE_AP) {
    Serial.println("Setting up Access Point...");
    bool result = WiFi.softAP("XRP-WPILib", "0123456789");
    if (result == true) {
      Serial.println("Ready");
    }
    else {
      Serial.println("Failed...");
    }
  }
  else {
    Serial.println("Connecting to AP");
    WiFi.begin("Meowza", "w1nthr0p");
    Serial.println("Connected?!");
  }

  hookupWSMessageHandlers();

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

void broadcast(std::string msg) {
  for (auto& clientPair : wsClients) {
    clientPair.second.send(msg.c_str());
  }
}

void onWsMessage(WebsocketsClient& client, WebsocketsMessage message) {
  if (message.isText()) {
    // if (message.data().indexOf("\"PWM\"") == -1 && message.data().indexOf("\"DriverStation\"") == -1) {
    //   return;
    // }
    // Process the message
    StaticJsonDocument<512> jsonDoc;
    DeserializationError error = deserializeJson(jsonDoc, message.data());
    if (error) {
      Serial.println(error.f_str());
      return;
    }

    // Hand this off to our processor
    wsMsgProcessor.processMessage(jsonDoc);

    if (jsonDoc.containsKey("type")) {
      messageCounts[jsonDoc["type"]]++;
    }
  }
}

void onWsEvent(WebsocketsClient& client, WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("Client Connection Closed");
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

// Main (CORE0) Loop
// This core should process WS messages and update the robot accordingly
void loop() {
  // Do Network Things
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

  while (rp2040.fifo.available()) {
    uint32_t data = rp2040.fifo.pop();

    if (data == ENCODER_DATA_AVAILABLE) {
      auto activeEncoders = robot.getActiveEncoderDeviceIds();
      for (auto& encId : activeEncoders) {
        int encVal = robot.getEncoderValueByDeviceId(encId);

        // Send the WS message
        auto jsonMsg = wsMsgProcessor.makeEncoderMessage(encId, encVal);
        broadcast(jsonMsg);
      }

    }
  }
}

void loop1() {
  // Read the encoders
  robot.periodic();

  delay(50);
}
