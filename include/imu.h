#pragma once

#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>

#include "watchdog.h"

#define GYRO_DATA_AVAILABLE 0xCC
#define ACCEL_DATA_AVAILBLE 0xDD



namespace xrp{
  class LSM6IMU {
    public:

      void init(uint8_t addr, TwoWire *theWire);
      bool isReady() { return _isReady; }
      void setEnabled(bool enabled) { _enabled = enabled; }

      void calibrate();

      void periodicOnCore1();
      void setReadLock(bool lock) { _readLock = lock; }

      void resetGyro();
      void resetAccel();

      float* getGyroRatesDegPerSec() { return _gyroRatesDegPerSec; }
      float* getGyroAnglesDeg() { return _gyroAnglesDeg; }
      float* getAccels() { return _accel; }

    private:
      bool _isReady;
      bool _enabled;
      Adafruit_LSM6DSOX _lsm6;

      float _gyroOffsetsDegPerSec[3];
      float _gyroRatesDegPerSec[3];
      float _gyroAnglesDeg[3];
      float _accel[3];

      unsigned long _lastUpdateTime;
      bool _onePassComplete = false;

      bool _readLock;
  };
}
