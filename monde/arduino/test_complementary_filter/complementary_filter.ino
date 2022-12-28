
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter

uint32_t timer;

#define DEG_TO_RAD 0.01745329238


float Yangle;

Adafruit_MPU6050 mpu;

/* Get new sensor events with the readings */
sensors_event_t a, g, temp;

void setup(void) {
  Serial.begin(250000);
  while (!Serial)
    delay(10); 
  Serial.setTimeout(100);
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }
                          
  delay(100);

  compAngleY = -90.0;
  compAngleX = 0.0;
  Yangle = 0.0;
  timer = micros();
 
  Serial.println("--START--");
}

float comp_angle() {
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  double roll  = atan(a.acceleration.y / sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;
  double pitch = atan2(-a.acceleration.x, a.acceleration.z) * RAD_TO_DEG;
  double gyroXrate = g.gyro.x * RAD_TO_DEG; // Convert to deg/s
  double gyroYrate = g.gyro.y * RAD_TO_DEG; // Convert to deg/s
  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  compAngleX = 0.98 * (compAngleX + gyroXrate * dt) + 0.02 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.98 * (compAngleY + gyroYrate * dt) + 0.02 * pitch;

  Yangle = compAngleY + 90;
}

void loop() {

  mpu.getEvent(&a, &g, &temp);
  comp_angle();

  Serial.print("angle : ");
  Serial.println(Yangle);
}
