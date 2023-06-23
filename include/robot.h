#pragma once

#include <unordered_map>
#include <vector>

#include "watchdog.h"

#define LEFT_MOTOR_EN 7
#define LEFT_MOTOR_PH 6
#define RIGHT_MOTOR_EN 15
#define RIGHT_MOTOR_PH 14
#define MOTOR_3_EN 3
#define MOTOR_3_PH 2
#define MOTOR_4_EN 11
#define MOTOR_4_PH 10
#define SERVO_1_PIN 16
#define SERVO_2_PIN 17

// These are from WPILib
#define ENCODER_L_CH_A 4
#define ENCODER_L_CH_B 5
#define ENCODER_R_CH_A 6
#define ENCODER_R_CH_B 7
#define ENCODER_3_CH_A 8
#define ENCODER_3_CH_B 9
#define ENCODER_4_CH_A 10
#define ENCODER_4_CH_B 11

#define ENCODER_CH_MOTOR_L 0
#define ENCODER_CH_MOTOR_R 1
#define ENCODER_CH_MOTOR_3 2
#define ENCODER_CH_MOTOR_4 3

// DIO - Physical pin mapping for built in things
#define DIO_BUILTIN_PHYS_USER_BUTTON 22
#define DIO_BUILTIN_PHYS_LED LED_BUILTIN

// Allowable Analog Pins 26 (ADC0), 27 (ADC1), 28 (ADC2)
// Onboard Temp Sensor - ADC4
// XRP Battery voltage - ADC2

#define ENCODER_DATA_AVAILABLE 0xAA
#define DIO_DATA_AVAILABLE 0xBB

#define DEFAULT_WATCHDOG_TIMEOUT_MS 1000

namespace xrp {
  class PWMChannel {
    public:
      PWMChannel() : _pin(-1) {}
      PWMChannel(int pin);
      virtual void setValue(double value);

    protected:
      int _pin;
  };

  class Motor : public PWMChannel {
    public:
      Motor(int enPin, int phPin);
      void setValue(double value);

    protected:
      int _enPin;
      int _phPin;
  };

  struct DIOChannel {
    bool isInput;
    int physicalPin;
    bool value;
  };

  class Robot {
    public:
      Robot();

      void configureEncoder(int deviceId, int chA, int chB);
      void configureDIO(int deviceId, bool isInput);

      void setEnabled(bool enabled);

      void setPwmValue(int channel, double value);
      void setDioValue(int channel, bool value);

      std::vector<int> getActiveEncoderDeviceIds();
      int getEncoderValueByDeviceId(int deviceId);
      int getEncoderValue(int idx);

      void checkStatus();

      // Periodic updates needed
      void periodicOnCore1();

      Watchdog watchdog{"robot"};

    private:
      bool _enabled;

      void setPwmValue(int channel, double value, bool override);
      
      // Encoder Values

      // Onboard Hardware
      Motor _leftMotor;
      Motor _rightMotor;
      Motor _motor3;
      Motor _motor4;
      PWMChannel _servo1;
      PWMChannel _servo2;

      // Channel Maps
      std::unordered_map<int, PWMChannel*> _pwmChannels;
      std::unordered_map<int, int> _encoderChannels; // Map from encoder device # to actual

      // Map from WPILib DIO -> Onboard DIO
      std::unordered_map<int, DIOChannel> _dioChannels;

      // Encoder Values
      int _encoderValues[4] = {0, 0, 0, 0};
  };


}
