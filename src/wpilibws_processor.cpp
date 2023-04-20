#include "wpilibws_processor.h"

namespace wpilibws {
  WPILibWSProcessor::WPILibWSProcessor() :
        _pwmCallback([](int, double){}),
        _dsGenericCallback([]() {}),
        _dsEnabledCallback([](bool) {}) {

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

  // Privates
  void WPILibWSProcessor::handlePWMMessage(JsonDocument& pwmMsg) {
    // Get the channel
    int channel = atoi(pwmMsg["device"].as<const char*>());
    auto data = pwmMsg["data"];

    // DEMO: We only care about the speed values (assuming PWM is init-ed)
    if (data.containsKey("<speed")) {
      double value = atof(data["<speed"].as<std::string>().c_str());
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
}