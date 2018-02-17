#include <Servo.h>

Servo brake;
Servo accelerator;
Servo gearLever;

int pos = 0;
int relayPin = 2;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(relayPin, OUTPUT);
  
  gearLever.attach(44);  // attaches the servo on pin 9 to the servo object
  accelerator.attach(45);
  brake.attach(46);
  
}

// the loop function runs over and over again forever
void loop() {
  relayOnOff(relayPin, HIGH);   // turn the LED on (HIGH is the voltage level)
  relayOnOff(relayPin, LOW);
  delay(2000);                       // wait for a second
  relayOnOff(relayPin, LOW);    // turn the LED off by making the voltage LOW
  relayOnOff(relayPin, HIGH);
  delay(2000);                       // wait for a second
}

bool relayOnOff(int relayPin, bool relayState)
{
  digitalWrite(relayPin, relayState);   
  // @param relayPin is a pin number to toggle
  // @param relayState is the state of the pin
}

void gears(int gearState)
{
  //@param gearState defines the gear position: park, reverse, drive
  //parking
  if (gearState == 1)
  {
    pos = 0;
    gearLever.write(pos);
  }
  //reverse
  else if (gearState == 2)
  {
    pos = 306;
    gearLever.write(pos);  
  }
  //drive
  else if (gearState == 3)
  {
    pos = 612;
    gearLever.write(pos);
  }
  else
  {
    return;
  }

  return;
}

void acceleration(int rawSpeed)
{
  //@param Speed is amount of acceleartion
  int outputSpeed = map(rawSpeed, 0, 40, 0, 1023);
  accelerator.write(outputSpeed); 
}

void braking(int rawBrake)
{
  //@param Speed is amount of acceleartion
  int outputBrake = map(rawBrake, 0, 40, 0, 1023);
  brake.write(outputBrake); 
}

void steering(int angle)
{
  int outputAngle = map(angle, 0, 3)  
}



