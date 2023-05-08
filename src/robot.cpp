#include "robot.h"
#include <Arduino.h>

namespace xrp {
  Robot::Robot() : 
      _enabled(false),
      _leftMotor(LEFT_MOTOR_EN, LEFT_MOTOR_PH),
      _rightMotor(RIGHT_MOTOR_EN, RIGHT_MOTOR_PH),
      _motor3(MOTOR_3_EN, MOTOR_3_PH),
      _motor4(MOTOR_4_EN, MOTOR_4_PH),
      _servo1(SERVO_1_PIN),
      _servo2(SERVO_2_PIN) {

    _pwmChannels.insert(std::make_pair(0, &_leftMotor));
    _pwmChannels.insert(std::make_pair(1, &_rightMotor));
    _pwmChannels.insert(std::make_pair(2, &_motor3));
    _pwmChannels.insert(std::make_pair(3, &_motor4));
    _pwmChannels.insert(std::make_pair(4, &_servo1));
    _pwmChannels.insert(std::make_pair(5, &_servo2));

    pinMode(LED_BUILTIN, OUTPUT);
  }

  void Robot::setEnabled(bool enabled) {
    this->_enabled = enabled;
    digitalWrite(LED_BUILTIN, enabled ? HIGH : LOW);

    // TODO if we're switching to disabled, reset all PWMs to 0
  }

  void Robot::setPwmValue(int channel, double value) {

    if (_pwmChannels.count(channel) > 0) {
      auto pwmChannel = _pwmChannels[channel];
      pwmChannel->setValue(value);
    }
  }

  // PWMChannel
  void PWMChannel::setValue(double value) {
    // Convert (-1.0, 1.0) to (0, 255)
    int val = ((value + 1.0) / 2.0) * 255;
    // Serial.println("PWMChannel setValue");
    analogWrite(_pin, val);
  }

  // Motor
  Motor::Motor(int enPin, int phPin) : 
          PWMChannel(-1), 
          _enPin(enPin), 
          _phPin(phPin) {
    pinMode(enPin, OUTPUT);
    pinMode(phPin, OUTPUT);
  }

  void Motor::setValue(double value) {
    // Serial.println("Motor setValue");
    PinStatus phValue = (value < 0.0) ? LOW : HIGH;
    int enValue = (abs(value) * 255);

    digitalWrite(_phPin, phValue);
    analogWrite(_enPin, enValue);
  }
}