/////////////////////////////////////////////////////////////////////////////////
// 
// Copyright Jean-Luc CHARLES (aka JLC) jean-luc.charles@member.fsfs.org
//           Alexis BOISSEAU
//
// JLC v1.0  initialised from balancingrobot_driver/driver1
// JLC v1.1  same command for Left and Right stepper motors
//           
//

#include <assert.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low  = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69

MPU6050 mpu;

/*
  port D : digital pins #0 to #7  on Arduino UNO board
  port B : digital pins #8 to #13 on Arduino UNO board
    (bits 6 & 7 of port B map to the crystal pins and are not available)
  
  PORTD,2 -> D2   PORTB,0 -> D8
  PORTD,3 -> D3   PORTB,1 -> D9
  PORTD,4 -> D4   PORTB,2 -> D10
  PORTD,5 -> D5   PORTB,3 -> D11
  PORTD,6 -> D6   PORTB,4 -> D12
  PORTD,7 -> D7   PORTB,5 -> D13
*/

///////////////////////////////////////////////////
///////      Pins Layout          /////////////////
///////////////////////////////////////////////////

/////////// STEPPER MOTOR //////////
#define PIN_DIR_R  5                                          
#define PIN_STEP_R 6                                         
#define PIN_DIR_L  3                                          
#define PIN_STEP_L 4                      

#define FORWARD   1
#define BACKWARD -1 

///////////////////////////////////////////////////
/////// stepper motors parameters /////////////////
///////////////////////////////////////////////////
#define fullStepAngle_deg 1.8                                 // the step angle of the stepper-motor
#define step_mode 4                                           // used for 1/2, 1/4, 1/8.... step mode

const float stepAngle_deg = fullStepAngle_deg/step_mode;      // the motor step angle in degrees
const float stepAngle_rad = stepAngle_deg*M_PI/180.;          // the motor step angle in radians
const int nbStepPerRevol  = int(360./stepAngle_deg);          // number of steps for a full revolution

///////////////////////////////////////////////////
//////////    Wheel  parameters   /////////////////
///////////////////////////////////////////////////
#define wheelDiameter_cm 9.5                                      // Wheel diameter [cm]
const float perimeter_cm = M_PI*wheelDiameter_cm;               // Wheel perimeter [cm]
const float stepDisp_cm  = stepAngle_rad*wheelDiameter_cm/2.;   // step distance [cm]
const float stepDisp_m   = stepDisp_cm*0.01;                    // step distance [m]

//
// global variables related to the stepper motor:
//
int MOTOR_DIR;                                                    // rotation direction FORWARD or BACKWARD
int TORQUE_ENABLED;                                               // used par ISR interrupt function to allow MOTOR_NB_STEP incrent/decrement.
const float max_Speed_cm_s   = 50 ;                               // max speed of the wheels midpoint 
// 80 cm/s -> 2.68 rps
// 70 cm/s -> 2.35 rps
// 60 cm/s -> 2.01 rps
// 50 cm/s -> 1.68 rps
// 40 cm/s -> 1.34 rps
// 30 cm/s -> 1.01 rps
const float max_motorSpeed_rps  = max_Speed_cm_s/perimeter_cm;    // max motor rps
const unsigned long int loop_dt_ms = 50;                          // wanted time loop
const unsigned long int loop_epsilon_ms = 1;                      // time lost in loop for calculus and  Serial.print

const float limit_angle_radians = 20.*M_PI/180.;

//
// global variables used by the timer/counter to generate the step signal
//
uint16_t counterPeriod_musec;       // the period of the timer counter in micro-secondes.
volatile uint16_t COUNTER;          // counters for periods
volatile long int MOTOR_NB_STEP;    // the number of steps done, to measure distance, duration...
uint16_t TARGET_COUNT;              // period to count for Left and Right motors

//
// global variables used by the MPU
//
float angle_gyro;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
bool flag_resetFIFO = false;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yp_offset = 0;


// interruption routine
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() 
{
    mpuInterrupt = true;
}

//
// Automaton states
//
enum AutomatonState
{
  WAIT,       // the balancing robot is not in a vertical position, PAS ENCORE UTILISE
  START,      // starts the balancing robot
  STOP        // stops the balancing robot
};
AutomatonState STATE; // the state of the driver autmaton
String stateLabel [] {String("[WAIT]"), String("[START]"), String("[STOP]")};

//
// glob vars to manage Balancing robot state :
//
float curr_xpos_m;            // current robot position [m]
float curr_lin_velocity;      // current robot linear velocity [m/s]
float curr_robot_angle_rad;         // current angle [rad] from MPU6050
float curr_robot_angle_velocity_rad;

float prev_xpos_m;            // previous robot position [m]
float prev_robot_angle_rad;         // previous angle [rad]

float observation[4];         // Defines the balancing robot state : [x, x', theta, theta']
bool DONE;                    // flag: used to end the episode
bool GO;                      // flag: used to start the episode

//
// miscellanous global variables:
//
String data = "";             // data received by ARDUINO on the USB link
float action ;                // the action infered by the neural network
float angle_rad;

//
// global vars to manage time values:
//
unsigned long int dt;                  // the loop time step
unsigned long int t0;
int timeDelay;

#define CLR(x,y) (x &= (~(1 << y)))
#define SET(x,y) (x |= (1 << y))

// Forward declaartion of run_test function to make the stepper motor move for testing...
inline void writeObservation();

inline void
setMotorSpeed_RPS(float speed_RPS) 
{
  // this function computes the value of 'TARGET_COUNT' necessary to make the stepper modor
  // rotate at the given speed (in Rotation Per Second)

  if (speed_RPS == 0.)   
  {
    TARGET_COUNT = 0;
  }
  else 
  {
    if (speed_RPS >= 0) 
    {
      MOTOR_DIR = FORWARD;
      CLR(PORTD, PIN_DIR_L);       // set pinDir to 0
      SET(PORTD, PIN_DIR_R);
    }
    else
    {
      speed_RPS = -speed_RPS;
      MOTOR_DIR = BACKWARD;
      SET(PORTD, PIN_DIR_L);      // set pinDir to 1
      CLR(PORTD, PIN_DIR_R);
    }
    // convert "revolution per second" into the number of tics timer to count 
    TARGET_COUNT = uint16_t(1.e6/(speed_RPS*nbStepPerRevol)/counterPeriod_musec);   // count step period 
  }
}

ISR(TIMER1_COMPA_vect) 
{
  //
  // Interrup function to generate the tops that drive the rotation speed of the stepper motor.
  //

  if (TARGET_COUNT == 0 || TORQUE_ENABLED == 0)
  {
    COUNTER = 0;
  }
  else
  {
    COUNTER++;
    if (COUNTER >= TARGET_COUNT)
    {
       
      SET(PORTD, PIN_STEP_L);         // STEP Motor LEFT
      SET(PORTD, PIN_STEP_R);         // STEP Motor RIGHT
      delayMicroseconds(2);
      CLR(PORTD, PIN_STEP_L);
      CLR(PORTD, PIN_STEP_R);
      
      MOTOR_NB_STEP += MOTOR_DIR;
      COUNTER = 0;
    }
  }  
}

inline void 
emptyUSB()
{
  // empty USB buffer, if needed:
  if (Serial.available()) {  
    // Data are available on the serial line (USB)
    while (Serial.available() > 0) {
      Serial.read();
    }    
  }
}


float get_angle()
{
  angle_rad = 999999;
  
  //if programming failed, don't try to do anything
  if (dmpReady)
  {
    mpu.resetFIFO(); 
  
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) { ; }
  
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
  
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
  
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
      // reset so we can continue cleanly
      //JLC_9sep21 mpu.resetFIFO();
      // Serial.println(F("FIFO overflow!"));
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    else if (mpuIntStatus & 0x02) 
    {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
          
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
  
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      angle_rad = ypr[1];
    }
  }
  return angle_rad;
}

void setup() 
{  

  //================
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  //================

  Serial.begin(250000); //Start the serial port to communicate with the master PC at 9600 kbps

  //================
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(17);  //JLC: was 220
  mpu.setYGyroOffset(31);  //JLC: was 76
  mpu.setZGyroOffset(-293);  //JLC: was -85
  mpu.setZAccelOffset(1391); //JLC: was 2201788// 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) 
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));     //DMP = Digital Motion Processor
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else 
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  //================

  //
  // To control the stepper motors we use Timer1 interrupt running at 25Khz.
  //
    
  cli(); // deactivate interruptions
  
  // TIMER1 CTC MODE (Clear Timer on Compare Match Mode)
  TCCR1A &= ~(1 << WGM10);   // set bit WGM10 to 0 in TCCR1A
  TCCR1A &= ~(1 << WGM11);   // set bit WGM11 to 0 in TCCR1A 
  TCCR1B |=  (1 << WGM12);   // set bit WGM12 to 1 in TCCR1B
  TCCR1B &= ~(1 << WGM13);   // set bit WGM13 to 0 in TCCR1B
  
  // output mode = 00 (disconnected)
  TCCR1A &= ~(3 << COM1A0); 
  TCCR1A &= ~(3 << COM1B0); 
  
  // Set the timer pre-scaler : we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR1B &= ~(0x07 << CS10);  // clear CS12, CS11 & CS10
  TCCR1B |=  (2 << CS10);     // set "CS12 CS11 CS10" to "0 1 0"
  
  // initalize counter value
  TCNT1 = 0;      
  
  // OCRIA with frequency 2 MHz:
  // 1000 ->   2 kHz : 0.500  ms
  //  500 ->   4 kHz : 0.250  ms
  //  250 ->   8 kHz : 0.125  ms
  //  125 ->  16 kHz : 0.0625 ms
  //  100 ->  20 kHz : 0.050  ms
  //   50 ->  40 kHz : 0.025  ms
  
  OCR1A = 50;   
  counterPeriod_musec = OCR1A/2;    // OCR1A / 2 MHz -> counter period in micro-sec
  
  // Enable Timer1 interrupt
  TIMSK1 |= (1 << OCIE1A);  
  
  sei(); // Activate interrupts
  

  // STEPPER PINS 
  pinMode(PIN_STEP_R, OUTPUT);   
  pinMode(PIN_STEP_L, OUTPUT);   
  pinMode(PIN_DIR_R,  OUTPUT);   
  pinMode(PIN_DIR_L,  OUTPUT);       

  setMotorSpeed_RPS(0);

 // calibration of the offset

  Serial.print(F("Computing yp_offset: keep the robot vertically "));
  int nb_value = 0;
  const int Nb = 3000;
  for (int i=0; i<Nb; i++) 
  {
    if(i % 25 == 0) 
    {
      digitalWrite(13, !digitalRead(13));
      Serial.print('.');
    }
    // wait for MPU interrupt or extra packet(s) available
    mpu.resetFIFO();
    while (!mpuInterrupt && fifoCount < packetSize) {;}
  
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
  
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
  
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    else if (mpuIntStatus & 0x02) 
    {
      if(i > 1000)
      {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
  
        // process MPU data 
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        yp_offset += ypr[1];
        nb_value++;
      }
    }
  }
  
  yp_offset /= nb_value;
  Serial.print(" done. yp_offset:"); Serial.println(yp_offset, 3);
  Serial.print(" (computed with nb_value="); Serial.println(nb_value);

  int debug = 1;
  {
    uint16_t min_targetCount  = uint16_t(1.e6/(max_motorSpeed_rps*nbStepPerRevol)/counterPeriod_musec);
    if (debug)
    {
      Serial.print("\n max robot speed    [cm/s]: "); Serial.print(max_Speed_cm_s);
      Serial.print("\n max motor speed     [rps]: "); Serial.print(max_motorSpeed_rps);
      Serial.print("\n nbStep per revolution    : "); Serial.print(nbStepPerRevol);
      Serial.print("\n counter period  [micro_s]: "); Serial.print(counterPeriod_musec);
      Serial.print("\n step displacement    [mm]: "); Serial.print(stepDisp_cm*10);
      Serial.print("\n displ. per revolution[cm]: "); Serial.print(nbStepPerRevol*stepDisp_cm);
      Serial.print("\n=== for max_motorSpeed ===");
      Serial.print("\n  TARGET_COUNT            : "); Serial.print(min_targetCount);      
      Serial.print("\n wanted time loop     [ms]: "); Serial.println(loop_dt_ms);
    }
  }  
  delay(1000);

  //
  // Hello with stepper motor...
  //
  if (true)
  {
    for (uint8_t i=0; i<4; i++)
    {
      setMotorSpeed_RPS(0.1);
      delay(50);
      setMotorSpeed_RPS(-0.1);
      delay(50);
    }
  }

  //run_test();
  
  setMotorSpeed_RPS(0.);

  //
  // Open the servo clamp
  //

  Serial.println("    Now place the robot vertically..and wait......................");
  
  TARGET_COUNT   = 0;
  COUNTER        = 0;
  MOTOR_NB_STEP  = 0;    
  TORQUE_ENABLED = 1;   
  DONE           = false;
  
  timeDelay      = 200;  
  STATE = WAIT;
}


///////////////////
// The Main loop //
///////////////////

void loop() 
{ 
  
  //////////////////////////////////////
  //   1. Processing Events           // 
  //////////////////////////////////////

  // Disable the motor torque to avoid damaging the mobile while exiting "START" state 
  setMotorSpeed_RPS(0.);

  //
  // At the end of an episode, DONE is set to false.
  //
  
  if (DONE) 
  {    
    //
    // switch to STOP state
    //
    STATE = STOP; 
    timeDelay = 200;              
    
    DONE  = false;         
   
    String m("End of the episode");     
    Serial.println(m);
  }
  else 
  {    
    if (STATE == WAIT)
    {
      STATE = START;
      Serial.print("Arduino OK\n");    // the key word to start episode!                    
    }

    //
    // switch state to START (start the balancing robot)
    //
    STATE = START;
    timeDelay = 0.;
    
    setMotorSpeed_RPS(0.);

    //
    // initialize the balancing robot state
    //
    curr_xpos_m           = 0.;  // current robot position [m]
    curr_lin_velocity     = 0.;  // current robot linear velocity [m/s]
    curr_robot_angle_rad  = 0.;  // current robot angle [rad]
    prev_xpos_m           = 0.;  // previous robot position [m]
    prev_robot_angle_rad  = 0.;  // previous robot angle [rad]                

    DONE          = false;
    TARGET_COUNT  = 0;
    COUNTER       = 0;
    MOTOR_NB_STEP = 0;       
    
    //
    // send first balancing robot observation (like the reset with Gym):
    //
    t0 = millis();
    dt = 0.;                              
    writeObservation();                            
  }
    
 
  //////////////////////////////////////
  // 2. Processing automaton state    // 
  //////////////////////////////////////

  if (STATE == START) 
  {
    
    Serial.setTimeout(5);
    
    while(! DONE) 
    {
      t0 = millis();
      DONE = false;
      
      //mpu.resetFIFO(); //JLC_9sept21
      
      //
      // read until data (motor speed) is available...
      //
      while (!Serial.available())   
      { ; }
  
      // we get the action given by the agent to the environment:
      // action is a float in the interval [-1. ; 1. ] must be multiplied by Vmax

      action = Serial.parseFloat();   // read the neural neutwork answer : action in [-1., 1.]
      emptyUSB();
      setMotorSpeed_RPS(action*max_motorSpeed_rps);

      angle_gyro = get_angle();
      while (angle_gyro == 999999)
      {
        angle_gyro = get_angle();
      }
      angle_gyro = yp_offset - angle_gyro;    // because the calibration of the robot is now done horizontally
      
      //
      // let the system run for a specified time (ms) :
      //
      while (millis() - t0 < loop_dt_ms -loop_epsilon_ms) 
      {
        delay(1);
      }  
      
      //
      // get the new system state:
      //
      curr_xpos_m    = MOTOR_NB_STEP*stepDisp_m;   // equivalent to R*angle
      curr_robot_angle_rad = angle_gyro;      
      if (fabs(curr_robot_angle_rad) > limit_angle_radians)  
      { 
        //DONE = true; 
      }
      dt = millis()-t0;  
      curr_lin_velocity = 1000.*(curr_xpos_m - prev_xpos_m)/dt;      
      curr_robot_angle_velocity_rad = 1000.*(curr_robot_angle_rad - prev_robot_angle_rad)/dt;
      
      writeObservation();     // Send observation (x,x',theta,theta',DONE) to the AGENT
      
      prev_xpos_m           = curr_xpos_m;            // x_{i-1} robot position [m]
      prev_robot_angle_rad  = curr_robot_angle_rad;   // theta_{i-1} robot angle [radian]      
    }
    Serial.setTimeout(1000);
  }

  delay(timeDelay);
}

inline void writeObservation()
{
  Serial.print(int(dt)); Serial.print(" ");
  Serial.print(curr_xpos_m, 3); Serial.print(" ");
  Serial.print(curr_lin_velocity, 3); Serial.print(" ");
  Serial.print(curr_robot_angle_rad, 2); Serial.print(" ");
  Serial.print(curr_robot_angle_velocity_rad, 2); Serial.print(" ");
  Serial.print(DONE); 
  Serial.print("\n");Serial.flush();
}
