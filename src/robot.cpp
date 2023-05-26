#include "robot.h"
#include <Arduino.h>
#include "quadrature_encoder.pio.h"
#include "quadrature.pio.h"

#define USER_BUTTON_PIN 22

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

    // TODO Prep I2C connection to gyro

    // Set up encoders
    uint offset0 = pio_add_program(pio0, &quadrature_program);
    quadrature_program_init(pio0, 0, offset0, 4, 5);
    quadrature_program_init(pio0, 1, offset0, 12, 13);
    quadrature_program_init(pio0, 2, offset0, 0, 1);
    quadrature_program_init(pio0, 3, offset0, 8, 9);

    pinMode(LED_BUILTIN, OUTPUT);

    // Set up the user button pin as input
    pinMode(USER_BUTTON_PIN, INPUT_PULLUP);
  }

  void Robot::setEnabled(bool enabled) {
    // TEMP: This prevents the motors from starting off with some arbitrary speed
    if (!this->_enabled && enabled) {
      setPwmValue(0, 0, true);
      setPwmValue(1, 0, true);
    }
    this->_enabled = enabled;
    // digitalWrite(LED_BUILTIN, enabled ? HIGH : LOW);

    // TODO if we're switching to disabled, reset all PWMs to 0
    // Assume that all PWMs are speed controllers, so 0 means no movement
  }

  void Robot::setPwmValue(int channel, double value) {
    setPwmValue(channel, value, false);
  }

  void Robot::setPwmValue(int channel, double value, bool override) {
    if (!this->_enabled && !override) {
      return;
    }

    if (_pwmChannels.count(channel) > 0) {
      auto pwmChannel = _pwmChannels[channel];
      pwmChannel->setValue(value);
    }
  }

  void Robot::setDioValue(int channel, bool value) {
    // TEMP
    if (channel == 1) {
      digitalWrite(LED_BUILTIN, value ? HIGH : LOW);
    }
  }

  void Robot::periodic() {
    // // Read off all the encoders
    for (int i = 0; i < 4; i++) {
      pio_sm_exec_wait_blocking(pio0, i, pio_encode_in(pio_x, 32));
      _encoderValues[i] = pio_sm_get_blocking(pio0, i);
    }

    Serial.print("[");
    Serial.print(get_core_num());
    Serial.print("] ");
    Serial.println("Encoder Read Complete");
    Serial.print(_encoderValues[0]);
    Serial.print(", ");
    Serial.println(_encoderValues[1]);
  }

  // PWMChannel
  PWMChannel::PWMChannel(int pin) : _pin(pin) {
    pinMode(pin, OUTPUT);
  }

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
