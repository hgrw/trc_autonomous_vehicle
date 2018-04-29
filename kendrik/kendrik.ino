/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
#define Brake_Out       5
#define Brake_In        3


Servo brakeServo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int pin = 0;

void setup() {
  Serial.begin(9600);
  brakeServo.attach(Brake_Out);
}

void loop() {
  Serial.println("Entering for loop");
  for (pos = 0; pos <= 1023; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    analogRead(pin);

    // 200 to 300
    if (pos > 200 && pos < 300) {
      Serial.println("200 < pos < 300");
    }

    brakeServo.write(pos);    // tell servo to go to position in variable 'pos'
    delay(15);                // waits 15ms for the servo to reach the position
  }
}

