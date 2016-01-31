int pushButton = 2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(pushButton, INPUT);
}

void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(A5);
  int sensorValue2 = analogRead(A4);
  int buttonState = digitalRead(pushButton);
  
  // print out the value you read:
  Serial.println(F("---------------------------------------------------------------------------------"));
  Serial.println(sensorValue);
  Serial.println(sensorValue2);
  Serial.println(buttonState);
  delay(2);        // delay in between reads for stability

}
