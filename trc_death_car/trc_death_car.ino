#include <Servo.h>

// Analog Inputs
#define Gear_Pot      A7
#define Steering_Pot  A1
#define Brake_Pot     A5

// Digital Outputs
#define Gear_Relay_Right  12
#define Gear_Relay_Left   11
#define Throttle_Out    7

// Gear constants
#define GEAR_P 540
#define GEAR_R 430
#define GEAR_N 360
#define GEAR_D 260
#define GEAR_TOLERANCE  7

// Brake constants
#define BRAKE_MIN  360
#define BRAKE_MAX   620 
#define BRAKE_TOLERANCE 10

// Throttle constants
#define THROTTLE_MIN     140
#define THROTTLE_MAX      40

// Steering constants           // TO FIX
#define STEER_MIN           295 // Right
#define STEER_MIDDLE        496 // Centre
#define STEER_MAX           780 //Left
#define STEER_TOLERANCE     3

// Init actuators
Servo throttleServo;

// misc vars
int gear_lever_state = GEAR_P;
int brake_state = BRAKE_MIN;
int steering_state = STEER_MIDDLE;

// Vars
int steer_data = 60;
int throttle_data = 0;
int brake_data = 0;
char gear_data = 'P';
int throttle_state = THROTTLE_MIN;
//int brake_state = BRAKE_MIN;
int steer_state = STEER_MIDDLE;
int brake_pot_last;


void setup()
{
  Serial.begin(9600);   // debug serial
  Serial3.begin(38400); // Large RoboClaw serial (steering)

  // set up pins
  pinMode(Gear_Relay_Right, OUTPUT);
  pinMode(Gear_Relay_Left, OUTPUT);
  pinMode(Brake_Pot, INPUT);
  pinMode(Gear_Pot, INPUT);
  pinMode(Steering_Pot, INPUT);

  // attach servos
  throttleServo.attach(Throttle_Out);

  // init actuators
  throttleServo.write(THROTTLE_MIN);
  
}


void loop()
{

  // listen for Jetson
  if (Serial.read() == 0xFF)
  {
    //Serial.println("Start loop");
    // wait for data
    while (!(Serial.available() > 3));
    
    //Serial.println("Test2");
    // Read data
    steer_data = Serial.read();
    throttle_data = Serial.read();
    brake_data = Serial.read();
    gear_data = Serial.read();

    Serial.print("Got ");
    Serial.print(steer_data);
    Serial.print(" ");
    Serial.print(throttle_data);
    Serial.print(" ");
    Serial.print(brake_data);
    Serial.print(" ");
    Serial.println(gear_data);
   
  }
  //Serial.println("Test2");
  setSteering((int)steer_data);
  setBrakes((int)brake_data);
  setGear((char)gear_data);
  setThrottle((int)throttle_data);
  

}

void setThrottle(int throttle_data)
{
  int newpos = THROTTLE_MIN + (float(throttle_data)/100.0) * (THROTTLE_MAX - THROTTLE_MIN);
  throttleServo.write(newpos);
}

void setSteering(int steer_data)
{
  // Convert to pot value
  int newpos = STEER_MIN + (float(steer_data) / 120.0) * (STEER_MAX - STEER_MIN);
  

  // RoboClaw Serial3 control, channel 2
  // 128 = full reverse
  // 192 = stop
  // 255 = full forward

  int oldpos = analogRead(Steering_Pot);
  //Serial.print("Pot Value");
  //Serial.println(oldpos);

  int error = int(float(abs(newpos - oldpos)) / 2.0);    // crude proportional steering rate control (makes it less twitchy)
  if ( error > 63 ) error = 63;             // then clamp value to max possible steering rate
  
  // Update motor
  if ( newpos <= (oldpos - STEER_TOLERANCE) )
  {
    // steer right
    Serial3.write(192 + error);
  }
  else if ( newpos > (oldpos + STEER_TOLERANCE) )
  {
    // steer left
    Serial3.write(192 - error);
  }
  else Serial3.write(192);  // else stop steering motor
}

void setBrakes(int brake_data)
{
  int newpos = BRAKE_MIN + (float(brake_data)/100.0) * (BRAKE_MAX - BRAKE_MIN);
 
  // RoboClaw Serial3 control, channel 2
  // 128 = full reverse
  // 192 = stop
  // 255 = full forward

  int oldpos = analogRead(Brake_Pot);
  
  //Serial.print("Brake Pot Value");
  //Serial.println(oldpos);

  int error = int(newpos - oldpos); 
  //int error = s));             // then clamp value to max possible steering rate
  
  if ( error < 0 - BRAKE_TOLERANCE )
  {
    // steer right
    Serial3.write(63 + 62);
  }
  else if (error > 0 + BRAKE_TOLERANCE  )
  {
    // steer left
    Serial3.write(63 - 62);
  }
  else Serial3.write(63);  // else stop steering motor
}

void setGear(char gear_char)
{
  int newpos = GEAR_N;
  if (gear_char == 'D')
  {
    newpos = GEAR_D;
  }
  else if (gear_char == 'R')
  {
    newpos = GEAR_R;
  }
  else if (gear_char == 'N')
  {
    newpos = GEAR_N;
  }
  else
  {
    newpos = GEAR_P;
  }
   

  int oldpos = analogRead(Gear_Pot);

//  brake_pot_last = oldpos;
  //Serial.print("Pot Value");
  //Serial.println(oldpos);
  //Serial.print("New Value");
  //Serial.print(newpos);
//  oldpos = 310; // N
  int error = int(newpos - oldpos);    
  //Serial.print("  Error:");
  //Serial.print(error);
  if ( error < 0 - GEAR_TOLERANCE )
  {
    // Retract actuator
    digitalWrite(Gear_Relay_Right, LOW);
    digitalWrite(Gear_Relay_Left, HIGH);
    //Serial.print("    IN");
  }
  else if ( error > 0 + GEAR_TOLERANCE)
  {
    // Extract actuator
    digitalWrite(Gear_Relay_Right, HIGH);
    digitalWrite(Gear_Relay_Left, LOW);
    //Serial.print("     OUT");
  }
  else  
  {
    // Stop
    digitalWrite(Gear_Relay_Right, LOW);
    digitalWrite(Gear_Relay_Left, LOW);
  } 
  
}

