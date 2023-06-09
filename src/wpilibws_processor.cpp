#include "wpilibws_processor.h"

namespace wpilibws {
  WPILibWSProcessor::WPILibWSProcessor() :
        _pwmCallback([](int, double){}),
        _dsGenericCallback([]() {}),
        _dsEnabledCallback([](bool) {}),
        _encoderInitCallback([](int, bool, int, int){}),
        _dioCallback([](int, bool){}) {

  }

  void WPILibWSProcessor::processMessage(JsonDocument& jsonMsg) {
    // Valid messages should have type, data and device fields
    if (jsonMsg.containsKey("type") && jsonMsg.containsKey("data")) {
      // Pass off to handlers
      if (jsonMsg["type"] == "PWM") {
        this->handlePWMMessage(jsonMsg);
      }
      else if (jsonMsg["type"] == "DriverStation") {
        this->handleDSMessage(jsonMsg);
      }
      else if (jsonMsg["type"] == "Encoder") {
        this->handleEncoderMessage(jsonMsg);
      }
      else if (jsonMsg["type"] == "DIO") {
        this->handleDIOMessage(jsonMsg);
      }
      else if (jsonMsg["type"] == "Gyro") {
        this->handleGyroMessage(jsonMsg);
      }
    }
  }

  // Callback Handlers
  void WPILibWSProcessor::onPWMMessage(PWMCallback callback) {
    this->_pwmCallback = callback;
  }

  void WPILibWSProcessor::onDSGenericMessage(DSGenericCallback callback) {
    this->_dsGenericCallback = callback;
  }

  void WPILibWSProcessor::onDSEnabledMessage(DSEnabledCallback callback) {
    this->_dsEnabledCallback = callback;
  }

  void WPILibWSProcessor::onEncoderInitMessage(EncoderInitCallback callback) {
    this->_encoderInitCallback = callback;
  }

  void WPILibWSProcessor::onDIOMessage(DIOCallback callback) {
    this->_dioCallback = callback;
  }

  void WPILibWSProcessor::onGyroInitMessage(GyroInitCallback callback) {
    this->_gyroInitCallback = callback;
  }

  // Message generators
  std::string WPILibWSProcessor::makeEncoderMessage(int deviceId, int count) {
    StaticJsonDocument<256> msg;
    msg["type"] = "Encoder";
    msg["device"] = std::to_string(deviceId);
    msg["data"][">count"] = count;

    std::string ret;
    serializeJson(msg, ret);
    return ret;
  }

  std::string WPILibWSProcessor::makeGyroMessage(float rates[3], float angles[3]) {
    StaticJsonDocument<400> msg;
    msg["type"] = "Gyro";
    msg["device"] = "RomiGyro";
    msg["data"][">rate_x"] = rates[0];
    msg["data"][">rate_y"] = rates[1];
    msg["data"][">rate_z"] = rates[2];
    msg["data"][">angle_x"] = angles[0];
    msg["data"][">angle_y"] = angles[1];
    msg["data"][">angle_z"] = angles[2];

    std::string ret;
    serializeJson(msg, ret);
    return ret;
  }

  // Privates
  void WPILibWSProcessor::handlePWMMessage(JsonDocument& pwmMsg) {
    // Get the channel
    int channel = atoi(pwmMsg["device"].as<const char*>());
    auto data = pwmMsg["data"];

    if (data.containsKey("<speed")) {
      // Speed values are [-1.0, 1.0]
      double value = atof(data["<speed"].as<std::string>().c_str());
      this->_pwmCallback(channel, value);
    }
    else if (data.containsKey("<position")) {
      // Position information is [0.0, 1.0]. We should convert to [-1.0, 1.0]
      double value = atof(data["<speed"].as<std::string>().c_str());
      value = (2.0 * value) - 1.0;
      this->_pwmCallback(channel, value);
    }
  }

  void WPILibWSProcessor::handleDSMessage(JsonDocument& dsMsg) {
    // Send the DSGenericCallback
    this->_dsGenericCallback();

    auto data = dsMsg["data"];
    if (data.containsKey(">enabled")) {
      this->_dsEnabledCallback(data[">enabled"].as<bool>());
    }
  }

  void WPILibWSProcessor::handleEncoderMessage(JsonDocument& encoderMsg) {
    int deviceNum = atoi(encoderMsg["device"].as<const char*>());
    auto data = encoderMsg["data"];

    if (data.containsKey("<init")) {
      bool initValue = data["<init"];
      int chA = -1;
      int chB = -1;

      if (data.containsKey("<channel_a")) {
        chA = data["<channel_a"];
      }
      if (data.containsKey("<channel_b")) {
        chB = data["<channel_b"];
      }

      this->_encoderInitCallback(deviceNum, initValue, chA, chB);
    }
  }

  void WPILibWSProcessor::handleDIOMessage(JsonDocument& dioMsg) {
    int channel = atoi(dioMsg["device"].as<const char*>());
    auto data = dioMsg["data"];

    if (data.containsKey("<>value")) {
      this->_dioCallback(channel, data["<>value"]);
    }
  }

  void WPILibWSProcessor::handleGyroMessage(JsonDocument& gyroMsg) {
    std::string gyroName = gyroMsg["device"];
    auto data = gyroMsg["data"];

    if (data.containsKey("<init")) {
      bool initValue = data["<init"];

      this->_gyroInitCallback(gyroName, initValue);
    }
  }
}
