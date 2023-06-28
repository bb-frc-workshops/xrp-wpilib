#include <Arduino.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

#include <WiFi.h>
#include <WiFiMulti.h>

#include <vector>

#include <Adafruit_LSM6DSOX.h>

#include <WebSockets4WebServer.h>
#include <WebServer.h>

#include "robot.h"
#include "imu.h"
#include "wpilibws_processor.h"
#include "config.h"
#include "watchdog.h"

// #define GYRO_DATA_AVAILABLE 0xCC
#define IMU_I2C_ADDR 0x6B

using namespace websockets;

XRPConfiguration config;

wpilibws::WPILibWSProcessor wsMsgProcessor;

xrp::Robot robot;
xrp::LSM6IMU imu;

xrp::Watchdog dsWatchdog{"ds"};

// Status Vars
NetworkMode netConfigResult;

// Chip Identifier
char chipID[20];

WebServer webServer(3300);
WebSockets4WebServer wsServer;

// ===================================================
// Handlers for INBOUND WS Messages
// ===================================================
void onDSGenericMessage() {
  // We use the DS messages to feed the watchdog
  robot.watchdog.feed();
  dsWatchdog.feed();
}

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
  wsMsgProcessor.onDSGenericMessage(onDSGenericMessage);
  wsMsgProcessor.onDSEnabledMessage(onDSEnabledMessage);
  wsMsgProcessor.onPWMMessage(onPWMMessage);
  wsMsgProcessor.onEncoderInitMessage(onEncoderInitMessage);
  wsMsgProcessor.onDIOMessage(onDIOMessage);
  wsMsgProcessor.onGyroInitMessage(onGyroInitMessage);
}

// ===================================================
// WebSocket management functions
// ===================================================

void broadcast(std::string msg) {
  wsServer.broadcastTXT(msg.c_str());
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected\n", num);
      break;
    case WStype_CONNECTED: {
        IPAddress ip = wsServer.remoteIP(num);
        Serial.printf("[%u] Connection from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      }
      break;
    case WStype_TEXT: {
        StaticJsonDocument<512> jsonDoc;
        DeserializationError error = deserializeJson(jsonDoc, payload);
        if (error) {
          Serial.println(error.f_str());
          break;
        }

        wsMsgProcessor.processMessage(jsonDoc);
      }
      break;
  }
}

// ===================================================
// Boot-Up and Main Control Flow
// ===================================================

void setup() {
  // Pick up the ChipID
  pico_unique_board_id_t id_out;
  pico_get_unique_board_id(&id_out);
  sprintf(chipID, "%02x%02x-%02x%02x", id_out.id[4], id_out.id[5], id_out.id[6], id_out.id[7]);

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

  // TODO Potentially set the WiFi SSID to include the last 4 bytes of unique_board_id
  Serial.print("ChipID: ");
  Serial.println(chipID);


  // Load configuration (and create default if one does not exist)
  config = loadConfiguration(std::string(chipID));

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("No WiFi Module");
    while (true);
  }

  WiFi.setHostname("XRP-Bot");
  // TODO mDNS setup?

  // Configure WiFi based off configuration
  netConfigResult = configureNetwork(config);
  Serial.print("Actual WiFi Mode: ");
  Serial.println(netConfigResult == NetworkMode::AP ? "AP" : "STA");

  // TODO Set up robot hardware overlays based off configuration

  // Set up WebSocket messages
  hookupWSMessageHandlers();

  // Set up the web server and websocket server hooks
  webServer.on("/", []() {
    webServer.send(200, "text/plain", "You probably want the websocket on /wpilibws\r\n");
  });

  webServer.addHook(wsServer.hookForWebserver("/wpilibws", onWebSocketEvent));

  webServer.begin();
  Serial.println("HTTP Server started on port 3300");
  Serial.println("WebSocket server started on /wpilibws on port 3300");
  Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
}

unsigned long lastStatusPrintTime = 0;

// Main (CORE0) Loop
// This core should process WS messages and update the robot accordingly
void loop() {
  // Robot Status
  robot.checkStatus();

  webServer.handleClient();
  wsServer.loop();

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
      // imu.setReadLock(true);
      auto jsonMsg = wsMsgProcessor.makeGyroMessage(imu.getGyroRatesDegPerSec(), imu.getGyroAnglesDeg());
      // imu.setReadLock(false);
      // broadcast(jsonMsg);
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
