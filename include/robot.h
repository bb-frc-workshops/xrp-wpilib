#pragma once

#include <unordered_map>

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
  
  class Robot {
    public:
      Robot();
      
      void setEnabled(bool enabled);
      
      void setPwmValue(int channel, double value);
      void setDioValue(int channel, bool value);

      // Periodic updates needed
      void periodic();

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
  };

  
}