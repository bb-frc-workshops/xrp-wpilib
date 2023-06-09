#include <Arduino.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

#include <WiFi.h>
#include <WiFiMulti.h>

#include <vector>

#include <Adafruit_LSM6DSOX.h>

#include "robot.h"
#include "imu.h"
#include "wpilibws_processor.h"
#include "config.h"

// #define GYRO_DATA_AVAILABLE 0xCC
#define IMU_I2C_ADDR 0x6B

using namespace websockets;

XRPConfiguration config;

WebsocketsServer server;
std::vector< std::pair<int, WebsocketsClient> > wsClients;
int nextClientId = 1;

wpilibws::WPILibWSProcessor wsMsgProcessor;

xrp::Robot robot;
xrp::LSM6IMU imu;


// ===================================================
// Handlers for INBOUND WS Messages
// ===================================================
void onDSEnabledMessage(bool enabled) {
  Serial.print("DS Enabled: ");
  Serial.println(enabled);
  robot.setEnabled(enabled);
}

void onPWMMessage(int channel, double value) {
  robot.setPwmValue(channel, value);
}

void onEncoderInitMessage(int channel, bool init, int chA, int chB) {
  if (init) {
    robot.configureEncoder(channel, chA, chB);
  }
}

void onDIOMessage(int channel, bool value) {
  robot.setDioValue(channel, value);
}

void onGyroInitMessage(std::string gyroName, bool enabled) {
  imu.setEnabled(enabled);
}

void hookupWSMessageHandlers() {
  // Hook up the event listeners to the message processor
  wsMsgProcessor.onDSEnabledMessage(onDSEnabledMessage);
  wsMsgProcessor.onPWMMessage(onPWMMessage);
  wsMsgProcessor.onEncoderInitMessage(onEncoderInitMessage);
  wsMsgProcessor.onDIOMessage(onDIOMessage);
  wsMsgProcessor.onGyroInitMessage(onGyroInitMessage);
}

// ===================================================
// WebSocket management functions
// ===================================================
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
    // Process the message
    StaticJsonDocument<512> jsonDoc;
    DeserializationError error = deserializeJson(jsonDoc, message.data());
    if (error) {
      Serial.println(error.f_str());
      return;
    }

    // Hand this off to our processor
    wsMsgProcessor.processMessage(jsonDoc);
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


// ===================================================
// Boot-Up and Main Control Flow
// ===================================================

void setup() {
  // Start up the File system and serial connections
  LittleFS.begin();
  Serial.begin(115200);

  // Set up the I2C pins
  Wire1.setSCL(19);
  Wire1.setSDA(18);
  Wire1.begin();

  // Delay a little to let i2c devices boot up
  delay(2000);

  // Initialize IMU
  imu.init(IMU_I2C_ADDR, &Wire1);
  imu.calibrate();

  // DEMO ONLY REMOVE BEFORE PRODUCTION USE
  // LittleFS.format();
  // delay(5000);

  // Load configuration (and create default if one does not exist)
  config = loadConfiguration();

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("No WiFi Module");
    while (true);
  }

  WiFi.setHostname("XRP-Bot");
  // TODO mDNS setup?

  // Configure WiFi based off configuration
  NetworkMode netConfigResult = configureNetwork(config);
  Serial.print("Actual WiFi Mode: ");
  Serial.println(netConfigResult == NetworkMode::AP ? "AP" : "STA");

  // TODO Set up robot hardware overlays based off configuration


  // Set up WebSocket messages
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

unsigned long lastStatusPrintTime = 0;

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
    else if (data == DIO_DATA_AVAILABLE) {
      // TODO Implement
    }
    else if (data == GYRO_DATA_AVAILABLE) {
      // TODO Send gyroReadings
      imu.setReadLock(true);
      auto jsonMsg = wsMsgProcessor.makeGyroMessage(imu.getGyroRatesDegPerSec(), imu.getGyroAnglesDeg());
      imu.setReadLock(false);
      broadcast(jsonMsg);
    }

  }

  delay(5);

  if (millis() - lastStatusPrintTime > 1000) {
    lastStatusPrintTime = millis();
    Serial.print("Cycle Count: ");
    Serial.print(rp2040.getCycleCount64());
    Serial.print(", Used Heap: ");
    Serial.println(rp2040.getUsedHeap());
  }
}

// Core 1 Loop
// This should essentially read sensors on a regular basis
void loop1() {
  // Read the encoders and other robot inputs
  robot.periodicOnCore1();

  // Read the IMU
  imu.periodicOnCore1();

  delay(50);
}
