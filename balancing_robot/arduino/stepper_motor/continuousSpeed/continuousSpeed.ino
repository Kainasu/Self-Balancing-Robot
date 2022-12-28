//
// JLC v1.0  2021/01/18  Driver for the Robot experience with continuous speed.
// JLC v1.1  2021/09/03  version for the two motors of the Balancing Robot

// **************
// ** WARNING : * 
// **************
// Do not run this programme with the motor of the CartPole bench => it wil move beyond the bench limits !!!!
//
// Use this programme with a free stepper moteur (linked to nothing).
//

#include <assert.h>

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
#define pinDir_L    3   // DIR_L
#define pinStep_L   4   // STEP_L 
#define pinDir_R    5   // DIR_R 
#define pinStep_R   6   // STEP_R

#define pinEnable 7   // ENABLE MOTOR
#define FORWARD     1   // Left
#define BACKWARD   -1   // Left


///////////////////////////////////////////////////
/////// stepper motors parameters /////////////////
///////////////////////////////////////////////////
#define fullStepAngle_deg 1.8                                       // the full step angle of the stepper-motor
#define step_mode 4                                                 // used for 1/2, 1/4, 1/8.... step mode
const float stepAngle_deg = fullStepAngle_deg/step_mode;            // the Left Right motors step angle
const float stepAngle_rad = stepAngle_deg*M_PI/180.;                // the Left Right motors step angle
const int nbStepPerRevol  = int(360./stepAngle_deg);                // number of steps for a full revolution

///////////////////////////////////////////////////
//////////    Wheel  parameters   /////////////////
///////////////////////////////////////////////////
#define wheelDiameter_cm  9                                     // Wheel diameter [cm]
const float perimeter_cm = M_PI*wheelDiameter_cm;               // Wheel perimeter [cm]
const float perimeter_m  = perimeter_cm*0.01;                   // Wheel perimeter [m]
const float stepDisp_cm  = stepAngle_rad*wheelDiameter_cm/2.;   // step distance [cm]
const float stepDisp_m   = stepDisp_cm*0.01;                    // step distance [m]

//
// global variables used by the timer/counter to generate the step signal
//
uint16_t counterPeriod_musec;           // the period of the timer counter in micro-secondes.
volatile uint16_t COUNTER;              // counters for periods
volatile long int MOTOR_NB_STEP;        // the number of steps done, to measure distance, duration...
uint16_t TARGET_COUNT;                   // period to count for motor

//
// global variables related to the stepper motor:
//
int MOTOR_DIR;                            // rotation direction FORWARD or BACKWARD
int TORQUE_ENABLED;                       // used par ISR interrupt function to allow MOTOR_NB_STEP incrent/decrement.
const float max_carSpeed_cm_s   = 50.;    // car speed [cm/s]
const float max_motorSpeed_rps  = max_carSpeed_cm_s/perimeter_cm;

//
// miscellanous global variables:
//
byte debug;           // the debug level (0 ... 20)

#define CLR(x,y) (x &= (~(1 << y)))
#define SET(x,y) (x |= (1 << y))
#define LF String("\n")

inline
void setMotorDir(int dir)
{
  // To set the direction of the motor:
  // TODO : must be configured following CopSim learning....
  // assert(dir== FORWARD || dir == BACKWARD);
  
  if (dir == FORWARD)
  {
    CLR(PORTD, pinDir_L);
    SET(PORTD, pinDir_R);
  }
  else
  { 
    SET(PORTD, pinDir_L);
    CLR(PORTD, pinDir_R);
  }
  MOTOR_DIR = dir;
}

inline void
setMotorSpeed_RPS(float speed_RPS) 
{
  // this function computes the value of 'TARGET_COUNT' necessary to make the stepper modor
  // rotate at the given speed (in Rotation Per Second)

  MOTOR_DIR = FORWARD;
  if (speed_RPS == 0.)   
  {
    TARGET_COUNT = 0;
  }
  else 
  {
    if (speed_RPS < 0) 
    {
      speed_RPS = -speed_RPS;
      MOTOR_DIR = BACKWARD;
    }
    // convert "revolution per second" into the number of tics timer to count 
    TARGET_COUNT = uint16_t(1.e6/(speed_RPS*nbStepPerRevol)/counterPeriod_musec);   // count step period 
    setMotorDir(MOTOR_DIR);
  }
}

inline void
enableTorque()
{
  CLR(PORTD, pinEnable);         // Enable motor torque, pin_7 -> LOW
  TORQUE_ENABLED = 1; 
}

inline void
disableTorque()
{
  SET(PORTD, pinEnable);         // Disable motor torque, pin_7 -> HIGH
  TORQUE_ENABLED = 0; 
}

//
// To control the stepper motors we use Timer1 interrupt running at 25Khz.
//

ISR(TIMER1_COMPA_vect)
{
  // Interrup function to generate the tops that drive the rotation speed of the stepper motor.
  
  if (TARGET_COUNT == 0 || TORQUE_ENABLED == 0)
  {
    COUNTER = 0;
  }
  else
  {
    COUNTER++;
    if (COUNTER >= TARGET_COUNT)
    {
      SET(PORTD, pinStep_L);         // STEP Motor Left
      CLR(PORTD, pinStep_L);
      SET(PORTD, pinStep_R);         // STEP Motor Right
      CLR(PORTD, pinStep_R);
      MOTOR_NB_STEP += MOTOR_DIR;
      COUNTER = 0;
    }
  }
}

void setup()
{  
  Serial.begin(115200);

  
  //
  // To control the stepper motors we use Timer1 interrupt running at 25Khz.
  //
  
  // STEPPER MOTORS INITIALIZATION
  
  cli(); // desactivate global interruption
  
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
  TCCR1B |=  (1 << CS11);     // set "CS12 CS11 CS10" to "0 1 0"

  // initalize counter value
  TCNT1 = 0;      

  // OCRIA with frequency 2 MHz:
  // 1000 ->   2 kHz : 0.500  ms
  //  500 ->   4 kHz : 0.250  ms
  //  250 ->   8 kHz : 0.125  ms
  //  125 ->  16 kHz : 0.0625 ms
  //  100 ->  20 kHz : 0.050  ms
  //   80 ->  25 kHz : 0.040 ms
  //   50 ->  40 kHz : 0.025  ms
  
  OCR1A = 50;   
  counterPeriod_musec = OCR1A/2;    // OCR1A / 2 MHz -> counter period in micro-sec

  // Enable Timer1 interrupt
  TIMSK1 |= (1 << OCIE1A);  

  sei(); // Activate global interrupt

  // STEPPER PINS 
  pinMode(pinEnable,OUTPUT);        // ENABLE MOTOR 
  pinMode(pinStep_L,  OUTPUT);      // STEP 
  pinMode(pinDir_L,   OUTPUT);      // DIR  
  pinMode(pinStep_R,  OUTPUT);      // STEP 
  pinMode(pinDir_R,   OUTPUT);      // DIR  

  disableTorque();   // Disable motor torque
  setMotorSpeed_RPS(0);

  uint16_t min_targetCount  = uint16_t(1.e6/(max_motorSpeed_rps*nbStepPerRevol)/counterPeriod_musec);
  debug = 1;
  if (debug)
  {
    Serial.print("\n max cart speed    [ cm/s]: "); Serial.print(max_carSpeed_cm_s);
    Serial.print("\n max motor speed     [rps]: "); Serial.print(max_motorSpeed_rps);
    Serial.print("\n nbStep per revolution    : "); Serial.print(nbStepPerRevol);
    Serial.print("\n counter period  [micro_s]: "); Serial.print(counterPeriod_musec);
    Serial.print("\n min targetCount          : "); Serial.print(min_targetCount);
    Serial.print("\n min_pulse period     [ms]: "); Serial.print(counterPeriod_musec*min_targetCount*1.e-3, 3);
    Serial.print("\n step displacement    [mm]: "); Serial.print(stepDisp_cm*10);
    Serial.print("\n disp. per revolution [cm]: "); Serial.println(nbStepPerRevol*stepDisp_cm);
  }
  
  if (true)
  {
    //
    // make the platform vibrate to say : "I'm alive...":
    //
    enableTorque();
    for (uint8_t i=0; i<4; i++)
    {
      setMotorSpeed_RPS(0.1);
      delay(50);
      setMotorSpeed_RPS(-0.1);
      delay(50);
    }
  }
  
  disableTorque();
  setMotorSpeed_RPS(0.);

  TARGET_COUNT  = 0;
  COUNTER       = 0;
  MOTOR_NB_STEP = 0;       // cumulated step number  
  
  delay(1000);
}

///////////////////
// The Main loop //
///////////////////

void loop() 
{     
  // Disabling the motor torque to avoid damaging the mobile 
  disableTorque();
  setMotorSpeed_RPS(0.);

  // create an increasing velocity ramp, a plate and then a decreasing ramp:
  enableTorque();   
  const int nb_speed_step = 1000;
  const float speed_increment = max_carSpeed_cm_s / nb_speed_step;
  for (float speed_cm_s = 0.; speed_cm_s < max_carSpeed_cm_s; speed_cm_s += speed_increment)
  {
    float motorSpeed_rps = speed_cm_s/perimeter_cm;
    setMotorSpeed_RPS(motorSpeed_rps);
    Serial.print("motorSpeed_cm/s: "); Serial.print(speed_cm_s);
    Serial.print("\tmotorSpeed_rps : "); Serial.print(motorSpeed_rps);
    Serial.print("\tTARGET_COUNT : "); Serial.println(TARGET_COUNT);
    delay(5);
  }
  for (float speed_cm_s = max_carSpeed_cm_s; speed_cm_s >=0; speed_cm_s -= speed_increment)
  {
    float motorSpeed_rps = speed_cm_s/perimeter_cm;
    setMotorSpeed_RPS(motorSpeed_rps);
    //Serial.print("motorSpeed_rps: "); Serial.print(motorSpeed_rps);
    //Serial.print("\tTARGET_COUNT: "); Serial.println(TARGET_COUNT);
    delay(2);
  }
  disableTorque();
  setMotorSpeed_RPS(0.);

  float motorSpeed_rps=2;
  setMotorSpeed_RPS(motorSpeed_rps);
  enableTorque();
  Serial.print("motorSpeed_rps : "); Serial.print(motorSpeed_rps);
  Serial.print("TARGET_COUNT   : "); Serial.println(TARGET_COUNT);
  Serial.print("step pulse [ms]   : "); Serial.println(counterPeriod_musec*TARGET_COUNT*1.e-3, 3);
  
  delay(50000);  

}
