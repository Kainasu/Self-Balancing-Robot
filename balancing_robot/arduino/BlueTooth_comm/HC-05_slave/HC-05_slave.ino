#include <SoftwareSerial.h>

#define rxPin 11    // Broche 11 en tant que RX, à raccorder sur TX du HC-05
#define txPin 10    // Broche 10 en tant que TX, à raccorder sur RX du HC-05
#define pinKey 12 

SoftwareSerial HC_05(rxPin, txPin);

void setup() 
{
  // define pin modes for tx, rx pins:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(pinKey, OUTPUT);
    
  // initialize serial communications at 9600 bps:
  Serial.begin(38400); 
  
  HC_05.begin(38400);
}

void loop() 
{
  //Serial.print("HC5 available: "); Serial.println(HC_05.available());
  if (HC_05.available())
  {
    byte received = HC_05.read();
    Serial.println(received);
    Serial.println("\nFIN");
  }
}
