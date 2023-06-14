#pragma once

#include <functional>
#include <ArduinoJson.h>

namespace wpilibws {
  typedef std::function<void(int channel, double value)> PWMCallback;
  typedef std::function<void()> DSGenericCallback;
  typedef std::function<void(bool dsEnabled)> DSEnabledCallback;
  typedef std::function<void(int device, bool enabled, int chA, int chB)> EncoderInitCallback;
  typedef std::function<void(int channel, bool value)> DIOCallback;
  typedef std::function<void(std::string devName, bool enabled)> GyroInitCallback;

  class WPILibWSProcessor {
    public:
      WPILibWSProcessor();
      void processMessage(JsonDocument& jsonMsg);

      void onPWMMessage(const PWMCallback callback);
      void onDSGenericMessage(const DSGenericCallback callback);
      void onDSEnabledMessage(const DSEnabledCallback callback);
      void onEncoderInitMessage(const EncoderInitCallback callback);
      void onDIOMessage(const DIOCallback callback);
      void onGyroInitMessage(const GyroInitCallback callback);

      std::string makeEncoderMessage(int deviceId, int count);
      std::string makeGyroMessage(float rates[3], float angles[3]);

    private:
      void handlePWMMessage(JsonDocument& pwmMsg);
      void handleDSMessage(JsonDocument& dsMsg);
      void handleEncoderMessage(JsonDocument& encoderMsg);
      void handleDIOMessage(JsonDocument& dioMsg);
      void handleGyroMessage(JsonDocument& gyroMsg);

      PWMCallback _pwmCallback;
      DSGenericCallback _dsGenericCallback;
      DSEnabledCallback _dsEnabledCallback;
      EncoderInitCallback _encoderInitCallback;
      DIOCallback _dioCallback;
      GyroInitCallback _gyroInitCallback;
  };
}
