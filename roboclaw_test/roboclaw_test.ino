//Roboclaw simple serial example.  Set mode to 6.  Option to 4(38400 bps)


void setup() {
//  mySerial.begin(38400);
  Serial2.begin(38400);
}

void loop() {
  delay(1000);
  Serial2.write(128);
  delay(1000);
  Serial2.write(192);
  delay(1000);
  Serial2.write(255);
  delay(1000);
  Serial2.write(192);
}
