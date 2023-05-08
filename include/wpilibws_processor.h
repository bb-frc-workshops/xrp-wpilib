#pragma once

#include <functional>
#include <ArduinoJson.h>

namespace wpilibws {
  typedef std::function<void(int channel, double value)> PWMCallback;
  typedef std::function<void()> DSGenericCallback;
  typedef std::function<void(bool dsEnabled)> DSEnabledCallback;

  class WPILibWSProcessor {
    public:
      WPILibWSProcessor();
      void processMessage(JsonDocument& jsonMsg);

      void onPWMMessage(const PWMCallback callback);
      void onDSGenericMessage(const DSGenericCallback callback);
      void onDSEnabledMessage(const DSEnabledCallback callback);

    private:
      void handlePWMMessage(JsonDocument& pwmMsg);
      void handleDSMessage(JsonDocument& dsMsg);
      void handleEncoderMessage(JsonDocument& encoderMsg);
      void handleDIOMessage(JsonDocument& dioMsg);

      PWMCallback _pwmCallback;
      DSGenericCallback _dsGenericCallback;
      DSEnabledCallback _dsEnabledCallback;
  };
}