
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <TimerOne.h>

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter

uint32_t timer;

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
///////////////////////////////////////////////////
///////      Pins Layout          /////////////////
///////////////////////////////////////////////////

/////////// STEPPER MOTOR //////////
#define pinDir_L    3   // DIR_L
#define pinStep_L   4   // STEP_L 
#define pinDir_R    5   // DIR_R 
#define pinStep_R   6   // STEP_R

              
///////////////////////////////////////////////////
/////// stepper motors parameters /////////////////
///////////////////////////////////////////////////
#define DEG_TO_RAD 0.01745329238


float Xangle = 0;
float action = 0;                // the action infered by the neural network

Adafruit_MPU6050 mpu;

/* Get new sensor events with the readings */
sensors_event_t a, g, temp;



#define FORWARD   1
#define BACKWARD -1 
#define wheelDiameter_mm 95    
int MOTOR_DIR;                                                    // rotation direction FORWARD or BACKWARD

int timerDelay = 100;
float speed_time = 100000.0;
float microstep = 4;
float degPerStep = 1.8/microstep;
float stepPerRotation = 360.0/degPerStep;
float distPerStep = degPerStep * DEG_TO_RAD * wheelDiameter_mm/2.0/1000.0;
unsigned long count = 0;
unsigned long MOTOR_NB_STEP = 0;

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
  pinMode(pinDir_L,OUTPUT);
  pinMode(pinStep_L,OUTPUT);
  pinMode(pinDir_R,OUTPUT);
  pinMode(pinStep_R,OUTPUT);

  Timer1.initialize(timerDelay);
  Timer1.attachInterrupt(callback);

  compAngleY = -90.0;
  compAngleX = 0.0;
  Xangle = 0.0;
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

  Xangle = compAngleY + 90;
}

#define turnOn() PORTD |=     B01010000
#define turnOff() PORTD &=    B10101111
#define turnDirOn() PORTD |=  B00001000; PORTD &= B11011111
#define turnDirOff() PORTD &= B11110111; PORTD |= B00100000

void callback()
{
    count++;
  if(count * timerDelay >=  speed_time){
    turnOn();
    delayMicroseconds(4);
    turnOff();
    MOTOR_NB_STEP += MOTOR_DIR;
    count=0;
  }
}

void loop() {

  while(!Serial.available()){
    ;
  }

  action = Serial.parseFloat();
  if(action > 1.0)
    action = 1.0; 
  if(action < -1.0) 
    action = -1.0;

  if(action == 0.0){
    speed_time = 1000000000.0;
  }else{
    speed_time = 1000000.0/(stepPerRotation*abs(2.0*action));
    if(action < 0.0 && MOTOR_DIR != BACKWARD){
        Timer1.stop();
        turnDirOff();
        MOTOR_DIR = BACKWARD;
        Timer1.start();
    }else if(action > 0.0 && MOTOR_DIR != FORWARD){
        Timer1.stop();
        turnDirOn();
        MOTOR_DIR = FORWARD;
        Timer1.start();
    }
  }
  mpu.getEvent(&a, &g, &temp);
  comp_angle();

  Serial.print(MOTOR_NB_STEP * distPerStep, 2);
  Serial.print("\t");
  Serial.print(stepPerRotation*2.0*action*distPerStep, 2);
  Serial.print("\t");
  Serial.print(Xangle * DEG_TO_RAD, 5);
  Serial.print("\t");
  Serial.println(g.gyro.y, 5);
}
