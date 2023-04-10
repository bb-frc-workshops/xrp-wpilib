#include "robot.h"
#include <Arduino.h>

namespace xrp {
  Robot::Robot() {
    pinMode(LEFT_MOTOR_PH, OUTPUT);
    pinMode(RIGHT_MOTOR_PH, OUTPUT);
  }

  void Robot::setPwmValue(int channel, double value) {
    // TODO This needs to be improved
    // Value is between -1.0 and 1.0
    if (channel < 2) {
      // We're referencing the motor drivers
      // channel 0 is left, channel 1 is right
      PinStatus ph = (value < 0.0) ? LOW : HIGH;
      int en = (abs(value) * 255);

      int phPin = (channel == 0) ? LEFT_MOTOR_PH : RIGHT_MOTOR_PH;
      int enPin = (channel == 0) ? LEFT_MOTOR_EN : RIGHT_MOTOR_EN;

      digitalWrite(phPin, ph);
      analogWrite(enPin, en);
    }
  }
}