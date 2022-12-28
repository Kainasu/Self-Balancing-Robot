#include <SoftwareSerial.h>

#define rxPin 11    // Broche 11 en tant que RX, à raccorder sur TX du HC-05
#define txPin 10    // Broche 10 en tant que TX, à raccorder sur RX du HC-05
#define pinKey 12 

SoftwareSerial HC_05(rxPin, txPin);

#define xPin  A1
#define yPin  A0
#define btnPin 2

int xPos = 0;
int yPos = 0;
int btnState = 0;

void setup() 
{
  // define pin modes for tx, rx pins:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(pinKey, OUTPUT);
  //activate pull-up resistor on the push-button pin
  pinMode(btnPin, INPUT_PULLUP); 
    
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  
  HC_05.begin(38400);
}

void loop() {
  xPos = analogRead(xPin);
  yPos = analogRead(yPin);
  btnState = digitalRead(btnPin);
  
  byte send_byte = B00000000;                                            //Set the send_byte variable to 0

  if(xPos < 500) send_byte |= B00000001; 
  if(xPos > 600) send_byte |= B00000010; 
  if(yPos < 500) send_byte |= B00001000; 
  if(yPos > 600) send_byte |= B00000100; 
  
  if(send_byte) 
  {
    Serial.print("X: "); Serial.print(xPos);
    Serial.print(" | Y: "); Serial.print(yPos);
    Serial.print(" | Button: "); Serial.println(btnState);
    
    HC_05.write(send_byte);                       
  }
  delay(50);              
}
