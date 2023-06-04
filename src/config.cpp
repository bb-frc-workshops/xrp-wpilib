#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <WiFiMulti.h>
#include <WiFi.h>

#include "config.h"

NetworkMode configureNetwork(XRPConfiguration config) {
  bool shouldUseAP = false;
  WiFiMulti multi;

  if (config.networkConfig.mode == NetworkMode::AP) {
    shouldUseAP = true;
  }
  else if (config.networkConfig.mode == NetworkMode::STA) {
    Serial.println("Attempting to start in STA Mode");
    Serial.println("Trying the following networks:");
    for (auto netInfo : config.networkConfig.networkList) {
      Serial.print("* ");
      Serial.println(netInfo.first.c_str());
      multi.addAP(netInfo.first.c_str(), netInfo.second.c_str());
    }

    // Attempt to connect
    if (multi.run() != WL_CONNECTED) {
      Serial.println("Failed to connect to any network on list. Falling back to AP");
      shouldUseAP = true;
    }
  }

  if (shouldUseAP) {
    Serial.println("Attempting to start in AP Mode");
    bool result = WiFi.softAP(
          config.networkConfig.defaultAPName.c_str(), 
          config.networkConfig.defaultAPPassword.c_str());

    if (result) {
      Serial.println("AP Ready");
    }
    else {
      Serial.println("AP Setup Failed");
    }
  }

  Serial.println("\n### Network Configured ###");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  return shouldUseAP ? NetworkMode::AP : NetworkMode::STA;
}

std::string generateDefaultConfig() {
  StaticJsonDocument<512> config;

  XRPConfiguration defaultConfig;
  
  // Networking API
  JsonObject network = config.createNestedObject("network");
  JsonObject defaultAP = network.createNestedObject("defaultAP");

  //Set up the default AP values
  defaultAP["ssid"] = defaultConfig.networkConfig.defaultAPName;
  defaultAP["password"] = defaultConfig.networkConfig.defaultAPPassword;

  JsonArray prefNetworks = network.createNestedArray("networkList");
  
  network["mode"] = defaultConfig.networkConfig.mode == NetworkMode::AP ? "AP" : "STA";

  // DEMO ONLY
  // JsonObject network1 = prefNetworks.createNestedObject();
  // network1["ssid"] = "Test Network 1";
  // network1["password"] = "Test Password";

  // JsonObject network2 = prefNetworks.createNestedObject();
  // network2["ssid"] = "Test Network 2";
  // network2["password"] = "Test Password";

  // network["mode"] = "STA";
  // END DEMO

  std::string ret;
  serializeJsonPretty(config, ret);
  return ret;
}


XRPConfiguration loadConfiguration() {
  XRPConfiguration config;

  File f = LittleFS.open("/config.json", "r");
  if (!f) {
    Serial.println("No config file found. Creating default");
    f = LittleFS.open("/config.json", "w");
    f.print(generateDefaultConfig().c_str());
    f.close();
  }

  f = LittleFS.open("/config.json", "r");
  
  // Load and verify
  StaticJsonDocument<512> configJson;
  auto jsonErr = deserializeJson(configJson, f);
  f.close();

  if (jsonErr) {
    Serial.print("Deserialization failed: ");
    Serial.println(jsonErr.f_str());
    return config;
  }

  // Network section
  if (configJson.containsKey("network")) {
    auto networkInfo = configJson["network"];

    // Check if there's a default AP provided
    if (networkInfo.containsKey("defaultAP")) {
      auto defaultAPInfo = networkInfo["defaultAP"];
      if (defaultAPInfo.containsKey("ssid")) {
        config.networkConfig.defaultAPName = defaultAPInfo["ssid"].as<std::string>();
      }
      if (defaultAPInfo.containsKey("password")) {
        config.networkConfig.defaultAPPassword = defaultAPInfo["password"].as<std::string>();
      }
    }

    // Load in the preferred network list
    if (networkInfo.containsKey("networkList")) {
      auto networkList = networkInfo["networkList"];
      JsonArray networks = networkList.as<JsonArray>();
      for (auto v : networks) {
        if (v.containsKey("ssid") && v.containsKey("password")) {
          config.networkConfig.networkList.push_back(std::make_pair<std::string, std::string>(v["ssid"], v["password"]));
        }
      }
    }

    // Check if we're in STA mode, and if so, we need at least 1 network
    if (networkInfo.containsKey("mode")) {
      if (networkInfo["mode"] == "STA" && config.networkConfig.networkList.size() > 0) {
        config.networkConfig.mode = NetworkMode::STA;
      }
      else {
        config.networkConfig.mode = NetworkMode::AP;
      }
    }
  }

  return config;
}