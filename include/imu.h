#pragma once

#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>

#define GYRO_DATA_AVAILABLE 0xCC
#define ACCEL_DATA_AVAILBLE 0xDD
#define NUM_CALIBRATION_SAMPLES 100


namespace xrp{
  class LSM6IMU {
    public:

      void init(uint8_t addr, TwoWire *theWire);
      bool isReady() { return _isReady; }
      void setEnabled(bool enabled) { _enabled = enabled; }
      void calibrate();

      void periodicOnCore1();

      void resetGyro();
      void resetAccel();

      float* getGyroRates() { return _gyroRates; }
      float* getGyroAngles() { return _gyroAngles; }
      float* getAccels() { return _accel; }

    private:
      bool _isReady;
      bool _enabled;
      Adafruit_LSM6DSOX _lsm6;

      float _gyroOffsets[3];
      float _gyroRates[3];
      float _gyroAngles[3];
      float _accel[3];

      unsigned long _lastUpdateTime;
      bool _onePassComplete = false;
  };
}
