//
// JLC v1.0 20211/01/22 Programm to test the A4988 driver.
//                      This chip is devoted to drive one stepper motor.
// JLC v1.1  2021/09/03  version for the two motors of the Balancing Robot
//

//
// GLOBAL variables
//

///////////////////////////////////////////////////
///////      Pins Layout          /////////////////
///////////////////////////////////////////////////
#define pinDir_L    3   // DIR_L
#define pinStep_L   4   // STEP_L 
#define pinDir_R    5   // DIR_R 
#define pinStep_R   6   // STEP_R

#define pinEnable 7     // ENABLE MOTOR
#define FORWARD     1   // Left
#define BACKWARD   -1   // Left

///////////////////////////////////////////////////
/////// stepper motors parameters /////////////////
///////////////////////////////////////////////////
#define fullStepAngle_deg 1.8                                       // the step angle of the stepper-motor
#define step_mode 4                                                 // used for 1/2, 1/4, 1/8.... step mode
const float stepAngle_deg = fullStepAngle_deg/step_mode;            // the Left Right motors step angle
const float stepAngle_rad = stepAngle_deg*M_PI/180.;                // the Left Right motors step angle
const int nbStepPerRevol  = int(360./stepAngle_deg);                // number of steps for a full revolution

int dir = 1;             // stepper rotation direction
bool done = false;
bool singleShot = false; // to choose singleshot or continuous mode

// macros usefull to write on digital pins:
#define CLR(x,y) (x &= (~(1 << y)))
#define SET(x,y) (x |= (1 << y))


void makeStepperTurn(int dir, 
                     float angle, 
                     float stepAngle, 
                     float nbRevolPerSec,  
                     bool releaseTorque = true)
{
  // dir          : stepper rotation direction (1 or -1)
  // angle        : motor rotation angle to do
  // stepAngle    : stepper motor angle (degrees) for a single step
  // nbRevolPerSec: stepper motor sped -> number of revolutions per second
  // releaseTorque: to realease torque or not after the rotation ?
  
  const float nbRevol   = 360./angle;
  const float timeDelay = 1000./(nbRevolPerSec*nbStepPerRevol);   // delay (ms) between 2 steps
  const long int totNbStep   = angle/stepAngle;
  
  Serial.print("RPS:");
  Serial.print(nbRevolPerSec);
  Serial.print(", timeDelay [ms]:");
  Serial.println(timeDelay);
  
  if (dir == 1) 
  {
     digitalWrite(pinDir_L, LOW) ;      // select rotation direction
     digitalWrite(pinDir_R, HIGH) ;     // select rotation direction
  } 
  else 
  {
    digitalWrite(pinDir_L, HIGH) ;      // select rotation direction
    digitalWrite(pinDir_R, LOW) ;     // select rotation direction
  }

  digitalWrite(pinEnable, LOW);  // apply torque
  
  for(long int i=0; i < totNbStep; i++)
  {
     SET(PORTD, pinStep_L);
     delayMicroseconds(2);
     CLR(PORTD, pinStep_L);
     
     SET(PORTD, pinStep_R);
     delayMicroseconds(2);
     CLR(PORTD, pinStep_R);
     
     delay(timeDelay);
  }
  if (releaseTorque) digitalWrite(pinEnable, HIGH);  // release torque
}

void setup()
{
  Serial.begin(115200);           // set up Serial link speed at 9600 bps
  Serial.println("Driver A4988 test begins...");
  Serial.print("Full step angle [°]: "); Serial.println(fullStepAngle_deg);
  Serial.print("Step mode          : 1/"); Serial.println(step_mode);
  Serial.print("Step angle      [°]: "); Serial.println(stepAngle_deg);
  Serial.print("Step angle     [rd]: "); Serial.println(stepAngle_rad);
  Serial.print("nb step per revol. : "); Serial.println(nbStepPerRevol);

  // STEPPER PINS 
  pinMode(pinEnable,OUTPUT);        // ENABLE MOTOR 
  pinMode(pinStep_L,  OUTPUT);      // STEP 
  pinMode(pinDir_L,   OUTPUT);      // DIR  
  pinMode(pinStep_R,  OUTPUT);      // STEP 
  pinMode(pinDir_R,   OUTPUT);      // DIR  

  digitalWrite(pinDir_L, LOW);
  digitalWrite(pinDir_R, HIGH);
  
  digitalWrite(pinEnable, HIGH); // disable torque
}

float speedRPS = 1;   // motor speed (Revolution Per Second)

void loop()
{
  dir = -dir;

  if (singleShot && !done)
  {
    for (speedRPS=0.5; speedRPS <= 2.; speedRPS += 0.25)
    {
      Serial.print("\nSpeedRPS [RPS]: "); Serial.println(speedRPS);
      makeStepperTurn(dir, 5*360., stepAngle_deg, speedRPS, true);
     delay(1000);
    }
    done = true;
  }
  else if (!singleShot)
  {
    Serial.print("\nSpeedRPS [RPS]: "); Serial.println(speedRPS);
    // if singleshot is not true, run makeStepperTurn every loop turn
    makeStepperTurn(dir, 50*360., stepAngle_deg, speedRPS, true);
    delay(1000);
  }
}
