// Jacek Fedorynski <jfedor@jfedor.org>
// http://www.jfedor.org/

#include <BLEHIDPeripheral.h>
#include "BLESerial.h"
#include <SparkFunMPU9250-DMP.h>

#define SDA_PIN 16
#define SCL_PIN 17

BLESerial BLESerial(-1, -1, -1);

MPU9250_DMP imu;

float mxmin = 1000000.0f;
float mxmax = -1000000.0f;
float mymin = 1000000.0f;
float mymax = -1000000.0f;
float mzmin = 1000000.0f;
float mzmax = -1000000.0f;

void setup() {
  Wire.setPins(SDA_PIN, SCL_PIN);
  imu.begin();

  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  imu.setSampleRate(100);
  imu.setCompassSampleRate(100);

  BLESerial.setLocalName("UART");
  BLESerial.begin();
}

void loop() {
  BLESerial.poll();

  if (imu.dataReady())
  {
    if (imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS) == INV_SUCCESS)
    {
      float mx = imu.calcMag(imu.mx);
      float my = imu.calcMag(imu.my);
      float mz = imu.calcMag(imu.mz);

      if (mx < mxmin) mxmin = mx;
      if (my < mymin) mymin = my;
      if (mz < mzmin) mzmin = mz;
      if (mx > mxmax) mxmax = mx;
      if (my > mymax) mymax = my;
      if (mz > mzmax) mzmax = mz;

      mx -= (mxmax + mxmin) * 0.5f;
      my -= (mymax + mymin) * 0.5f;
      mz -= (mzmax + mzmin) * 0.5f;

      BLESerial.print((mxmax + mxmin) * 0.5f, 6); BLESerial.print(" ");
      BLESerial.print((mymax + mymin) * 0.5f, 6); BLESerial.print(" ");
      BLESerial.print((mzmax + mzmin) * 0.5f, 6); BLESerial.println();
    }
  }
}
