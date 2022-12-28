//
// JLC v1.0 20211/01/22 Programm to test the A4988 driver.
//                      This chip is devoted to drive one stepper motor.
//

//
// GLOBAL variables
//

#include <TimerOne.h>

///////////////////////////////////////////////
// pins for the stepper motor driver A4988   //
///////////////////////////////////////////////
const int pinDir    = 5;  // pin for direction of rotation
const int pinStep   = 6;  // pin for stepping
const int pinEnable = 7;  // pin for Enable/disable torque

///////////////////////////////////////////////////
/////// stepper motors parameters /////////////////
///////////////////////////////////////////////////
#define fullStepAngle_deg 1.8                                       // the step angle of the stepper-motor

#define step_mode 4                                                 // used for 1/2, 1/4, 1/8.... step mode

const float stepAngle_deg = fullStepAngle_deg/step_mode;            // the Left Right motors step angle
const float stepAngle_rad = fullStepAngle_deg/step_mode*M_PI/180.;  // the Left Right motors step angle
const int nbStepPerRevol  = int(360./stepAngle_deg);                // number of steps for a full revolution

int dir = 1;             // stepper rotation direction
bool done = false;
bool singleShot = true; // to choose singleshot or continuous mode

// macros usefull to write on digital pins:
#define CLR(x,y) (x &= (~(1 << y)))
#define SET(x,y) (x |= (1 << y))


int timerDelay = 100;
float stepDelay;
int count;
void callback()
{
  count++;
  if(count * timerDelay >=  stepDelay){
    SET(PORTD, pinStep);
    delayMicroseconds(2);
    CLR(PORTD, pinStep);
    count=0;
  }
}

void makeStepperTurn(int dir, float nbRevolPerSec)
{
  const float timeDelay = 1000000./(nbRevolPerSec*nbStepPerRevol);   // delay (ns) between 2 steps
  Serial.print("RPS:");
  Serial.print(nbRevolPerSec);
  Serial.print(", timeDelay [ms]:");
  Serial.println(timeDelay);
  
  if (dir == 1) 
  {
      Timer1.stop();
      CLR(PORTD, pinDir);
      Timer1.start();
  } 
  else 
  {
      Timer1.stop();
      SET(PORTD, pinDir);
      Timer1.start();
  }
  stepDelay = timeDelay;  
}

void setup()
{
  Serial.begin(115200);           // set up Serial link speed at 9600 bps
  Serial.println("\n\nDriver A4988 test begins...");
  Serial.print("Full step angle [°]: "); Serial.println(fullStepAngle_deg);
  Serial.print("Step mode          : 1/"); Serial.println(step_mode);
  Serial.print("Step angle      [°]: "); Serial.println(stepAngle_deg);
  Serial.print("Step angle     [rd]: "); Serial.println(stepAngle_rad);
  Serial.print("nb step per revol. : "); Serial.println(nbStepPerRevol);

  // set up the switch pin as an input and Leds as output
  pinMode(pinDir, OUTPUT);
  pinMode(pinStep, OUTPUT);
  pinMode(pinEnable, OUTPUT);

  digitalWrite(pinDir, LOW);
  digitalWrite(pinEnable, HIGH); // disable torque
  
  Timer1.initialize(timerDelay);
  Timer1.attachInterrupt(callback);

}

float speedRPS = 1.;   // motor speed (Revolution Per Second)

void loop()
{
  dir = -dir;

  if (singleShot && !done)
  {
    for (speedRPS=0.5; speedRPS <= 2.; speedRPS += 0.25)
    {
      Serial.print("\nSpeedRPS [RPS]: "); Serial.println(speedRPS);
      makeStepperTurn(dir, speedRPS);
     delay(5000);
    }
    done = true;
  }
  else if (!singleShot)
  {
    Serial.print("\nSpeedRPS [RPS]: "); Serial.println(speedRPS);
    // if singleshot is not true, run makeStepperTurn every loop turn
    makeStepperTurn(dir, speedRPS);
    delay(1000);
  }
}
