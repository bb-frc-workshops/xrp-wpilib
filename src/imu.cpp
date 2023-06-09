#include "imu.h"
#include <limits>

namespace xrp {

  float radToDeg(float angleRad) {
    return angleRad * 180.0 / PI;
  }

  void LSM6IMU::init(uint8_t addr, TwoWire *theWire) {
    if (!_lsm6.begin_I2C(addr, theWire, 0)) {
      Serial.println("Failed to find LSM6DSOX");
      _isReady = false;
    }
    else {
      _isReady = true;
      Serial.println("--- IMU ---");
      Serial.println("LSM6DSOX detected");
      Serial.print("Accel Range: ");
      switch (_lsm6.getAccelRange()) {
        case LSM6DS_ACCEL_RANGE_2_G:
          Serial.println("+-2G");
          break;
        case LSM6DS_ACCEL_RANGE_4_G:
          Serial.println("+-4G");
          break;
        case LSM6DS_ACCEL_RANGE_8_G:
          Serial.println("+-8G");
          break;
        case LSM6DS_ACCEL_RANGE_16_G:
          Serial.println("+-16G");
          break;
      }

      Serial.print("Gyro Range: ");
      switch(_lsm6.getGyroRange()) {
        case LSM6DS_GYRO_RANGE_125_DPS:
          Serial.println("125 DPS");
          break;
        case LSM6DS_GYRO_RANGE_250_DPS:
          Serial.println("250 DPS");
          break;
        case LSM6DS_GYRO_RANGE_500_DPS:
          Serial.println("500 DPS");
          break;
        case LSM6DS_GYRO_RANGE_1000_DPS:
          Serial.println("1000 DPS");
          break;
        case LSM6DS_GYRO_RANGE_2000_DPS:
          Serial.println("2000 DPS");
          break;
        case ISM330DHCX_GYRO_RANGE_4000_DPS:
          break;
      }
    }
  }

  void LSM6IMU::calibrate() {
    if (!_isReady) return;

    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::min();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::min();
    float minZ = std::numeric_limits<float>::max();
    float maxZ = std::numeric_limits<float>::min();

    float totalX = 0;
    float totalY = 0;
    float totalZ = 0;

    bool ledOn = false;
    unsigned long lastSwitched = 0;

    for (int i = 0; i < NUM_CALIBRATION_SAMPLES; i++) {
      if (millis() - lastSwitched > 100) {
        ledOn = !ledOn;
        digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
        lastSwitched = millis();
      }

      sensors_event_t accel;
      sensors_event_t gyro;
      sensors_event_t temp;

      _lsm6.getEvent(&accel, &gyro, &temp);

      float rateX = radToDeg(gyro.gyro.x);
      float rateY = radToDeg(gyro.gyro.y);
      float rateZ = radToDeg(gyro.gyro.z);

      minX = min(minX, rateX);
      maxX = max(maxX, rateX);
      minY = min(minY, rateY);
      maxY = max(maxY, rateY);
      minZ = min(minZ, rateZ);
      maxZ = max(maxZ, rateZ);

      totalX += rateX;
      totalY += rateY;
      totalZ += rateZ;

      delay(50);
    }

    _gyroOffsetsDegPerSec[0] = totalX / NUM_CALIBRATION_SAMPLES;
    _gyroOffsetsDegPerSec[1] = totalY / NUM_CALIBRATION_SAMPLES;
    _gyroOffsetsDegPerSec[2] = totalZ / NUM_CALIBRATION_SAMPLES;

    Serial.println("Calibration Complete");
    Serial.print("Offsets: X(");
    Serial.print(_gyroOffsetsDegPerSec[0]);
    Serial.print("), Y(");
    Serial.print(_gyroOffsetsDegPerSec[1]);
    Serial.print("), Z(");
    Serial.print(_gyroOffsetsDegPerSec[2]);
    Serial.println(")");
    Serial.print("Noise: X(");
    Serial.print(maxX - minX);
    Serial.print("), Y(");
    Serial.print(maxY - minY);
    Serial.print("), Z(");
    Serial.print(maxZ - minZ);
    Serial.println(")");

    digitalWrite(LED_BUILTIN, LOW);
  }

  void LSM6IMU::periodicOnCore1() {
    if (!_isReady) return;
    if (!_enabled) return;
    if (get_core_num() != 1) return;

    if (_readLock) {
      Serial.print("READ LOCK - ");
      Serial.println(millis());
      return;
    }

    unsigned long currTime = millis();

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    _lsm6.getEvent(&accel, &gyro, &temp);

    float rateX = radToDeg(gyro.gyro.x);
    float rateY = radToDeg(gyro.gyro.y);
    float rateZ = radToDeg(gyro.gyro.z);
    _gyroRatesDegPerSec[0] = rateX - _gyroOffsetsDegPerSec[0];
    _gyroRatesDegPerSec[1] = rateY - _gyroOffsetsDegPerSec[1];
    _gyroRatesDegPerSec[2] = rateZ - _gyroOffsetsDegPerSec[2];

    if (!_onePassComplete) {
      _onePassComplete = true;
    }
    else {
      unsigned long dt = currTime - _lastUpdateTime;
      float dtInSeconds = dt / 1000.0;

      _gyroAnglesDeg[0] = _gyroAnglesDeg[0] + (_gyroRatesDegPerSec[0] * dtInSeconds);
      _gyroAnglesDeg[1] = _gyroAnglesDeg[1] + (_gyroRatesDegPerSec[1] * dtInSeconds);
      _gyroAnglesDeg[2] = _gyroAnglesDeg[2] + (_gyroRatesDegPerSec[2] * dtInSeconds);
    }

    _lastUpdateTime = currTime;

    rp2040.fifo.push(GYRO_DATA_AVAILABLE);
  }

  void LSM6IMU::resetGyro() {
    for (int i = 0; i < 3; i++) {
      _gyroAnglesDeg[i] = 0;
      _gyroRatesDegPerSec[i] = 0;
    }
  }

  void LSM6IMU::resetAccel() {
    _accel[0] = 0;
    _accel[1] = 0;
    _accel[2] = 0;
  }
}
