// Jacek Fedorynski <jfedor@jfedor.org>
// http://www.jfedor.org/

#include <BLEHIDPeripheral.h>
#include <BLEMouse.h>
#include <SparkFunMPU9250-DMP.h>
#include <Adafruit_AHRS.h>

#define SENSITIVITY 96
#define THRESHOLD 0.05

#define SDA_PIN 16
#define SCL_PIN 17
#define RED_LED_PIN 19
#define GREEN_LED_PIN 20
#define BLUE_LED_PIN 21
#define UNPAIR_PIN 28
#define GROUND_PIN 25

// these values are board-specific and need to be calibrated
#define MAG_X_BIAS -21.60
#define MAG_Y_BIAS -9.07
#define MAG_Z_BIAS -36.75

BLEHIDPeripheral bleHIDPeripheral = BLEHIDPeripheral(-1, -1, -1);
BLEMouse bleMouse;

MPU9250_DMP imu;
Adafruit_Madgwick filter;
//Adafruit_Mahony filter;
//Adafruit_NXPSensorFusion filter;

bool blinkState = false;
float prevquat[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
float rotationZ = 0.0f;
float x = 0.0f;
float y = 0.0f;

void setup() {
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, HIGH);
  pinMode(BLUE_LED_PIN, OUTPUT);
  digitalWrite(BLUE_LED_PIN, HIGH);

  pinMode(UNPAIR_PIN, INPUT_PULLUP);
  pinMode(GROUND_PIN, OUTPUT);
  digitalWrite(GROUND_PIN, LOW);

  filter.begin(50); // Hz
  Wire.setPins(SDA_PIN, SCL_PIN);
  imu.begin();

  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  imu.setSampleRate(50);
  imu.setCompassSampleRate(50);

  bleHIDPeripheral.setLocalName("Human Trackball");
  bleHIDPeripheral.setDeviceName("Human Trackball");
  bleHIDPeripheral.setAppearance(962); // doesn't seem to work on mac
  bleHIDPeripheral.addHID(bleMouse);

  bleHIDPeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  bleHIDPeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  if (digitalRead(UNPAIR_PIN) == LOW) {
    bleHIDPeripheral.clearBondStoreData();
  }

  bleHIDPeripheral.begin();
}

// quaternion multiplication
void quatmul(float* result, float* r, float* q) {
  result[0] = r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3];
  result[1] = r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2];
  result[2] = r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1];
  result[3] = r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0];
}

void mouseCommand(int8_t x, int8_t y) {
  bleMouse.move(x, y);
}

void blePeripheralConnectHandler(BLECentral& central) {
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(BLUE_LED_PIN, LOW);
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, HIGH);
}

int sgn(float x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}

void loop() {
  BLECentral central = bleHIDPeripheral.central();
  if ( central && central.connected() && imu.dataReady() )
  {
    if (imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS) == INV_SUCCESS)
    {
      float ax = imu.calcAccel(imu.ax);
      float ay = imu.calcAccel(imu.ay);
      float az = imu.calcAccel(imu.az);
      float gx = imu.calcGyro(imu.gx);
      float gy = imu.calcGyro(imu.gy);
      float gz = imu.calcGyro(imu.gz);
      float mx = imu.calcMag(imu.mx);
      float my = imu.calcMag(imu.my);
      float mz = imu.calcMag(imu.mz);

      mx -= MAG_X_BIAS;
      my -= MAG_Y_BIAS;
      mz -= MAG_Z_BIAS;

      // perform the sensor fusion magic
      // axis orientation on the MPU-9250 is different for the magnetometer than for the accelerometer and gyro
      filter.update(gx, gy, gz, ax, ay, az, my, mx, -mz);

      float quat[4];
      filter.getQuaternion(&quat[0], &quat[1], &quat[2], &quat[3]);

      // calculate the difference between previous and current quaternion
      float dquat[4];
      for (int i = 1; i < 4; i++) {
        prevquat[i] *= -1;
      }
      quatmul(dquat, prevquat, quat);

      for (int i = 0; i < 4; i++) {
        prevquat[i] = quat[i];
      }

      // convert quaternion to axis-angle
      // https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm

      float s = sqrt(1 - dquat[0] * dquat[0]);

      if (!isnan(s) && s > 0.001) {
        float angle = 2.0 * acos(dquat[0]);
        // we want the angle between -180 and 180 degrees
        if (angle > PI) {
          angle = PI - angle;
        }

        float rotationX = angle * dquat[1] / s;
        float rotationY = angle * dquat[2] / s;
        rotationZ += angle * dquat[3] / s;

        // rotate around Z axis using the accumulated angle to let the user set the "up" direction
        float sinz = sin(rotationZ);
        float cosz = cos(rotationZ);
        // rotation around X axis moves the cursor along Y axis
        y += rotationX * cosz - rotationY * sinz;
        x += rotationX * sinz + rotationY * cosz;
      }

      // ignore the readings for the first 10 seconds
      if (millis() < 10000) {
        x = 0.0f;
        y = 0.0f;
      }

      int8_t int_x = fabs(x) > THRESHOLD ? (x - sgn(x)*THRESHOLD) * SENSITIVITY : 0;
      int8_t int_y = fabs(y) > THRESHOLD ? (y - sgn(y)*THRESHOLD) * SENSITIVITY : 0;

      if (int_x != 0 || int_y != 0) {
        mouseCommand(int_x, int_y);
        blinkState = !blinkState;
        digitalWrite(GREEN_LED_PIN, blinkState);
      }
    }
  }
}
