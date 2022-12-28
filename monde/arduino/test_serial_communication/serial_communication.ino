
void setup(void) {
  Serial.begin(250000);
  while (!Serial)
    delay(10); 
  Serial.setTimeout(100);

  delay(100);
  Serial.println("--START--");
}


void loop() {

  while(!Serial.available()){
    ;
  }

  float action = Serial.parseFloat();
  Serial.println(action);
}
