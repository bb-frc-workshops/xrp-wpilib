#pragma once

#define LEFT_MOTOR_EN 7
#define LEFT_MOTOR_PH 6
#define RIGHT_MOTOR_EN 15
#define RIGHT_MOTOR_PH 14

namespace xrp {
  class Robot {
    public:
      Robot();
      void setPwmValue(int channel, double value);
  };
}