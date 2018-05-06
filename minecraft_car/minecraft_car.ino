#include <Servo.h>
#include <PID_v1.h>

// states for state machine
#define PARK        0
#define START_CAR   1
#define NEUTRAL_RC  2
#define DRIVE_RC    3
#define DRIVE_AI    4
#define REVERSE_RC  5
#define STOP_CAR    6

// Arduino pins
#define RC_CH_2     6 // throttle/brake (left stick, y axis)
#define RC_CH_1     2 // steering (right stick, x axis)

#define Gear_Pot      A7
#define Steering_Pot  A1
#define Brake_Pot     A5

#define Ignition_Relay  12
#define Battery_Relay   11
// #define Jetson_Boot     39
#define Throttle_Out    7
#define Brake_Dir_Out   34
#define Brake_PWM_Out   36
#define Gear_Dir_Out    52
#define Gear_PWM_Out    48
#define Steering_Out    4

// gear constants
#define GEAR_P 0
#define GEAR_R 1
#define GEAR_N 2
#define GEAR_D 3

#define GEAR_TOLERANCE  5

// brake constants
#define BRAKE_ZERO  0
#define BRAKE_MAX   180
#define BRAKE_TOLERANCE 5

// throttle constants
#define THROTTLE_ZERO     170
#define THROTTLE_LOW      160
#define THROTTLE_SENSIBLE 120
#define THROTTLE_MAX      90

// steering constants
#define STEER_MIN           300
#define STEER_MIDDLE        600
#define STEER_MAX           1023
#define STEER_TOLERANCE     10

// misc constants
#define CRANK_TIME 1000

// init actuators
Servo throttleServo;

// init state machine
int state = PARK;

// misc vars
int gear_lever_state = GEAR_P;
int brake_state = BRAKE_ZERO;
int steering_state = STEER_MIDDLE;
int start_delay_counter = 0;

// RC vars
int rc_throttle = THROTTLE_ZERO;
int rc_brake = BRAKE_MAX;
int rc_gear = GEAR_P;
int rc_steering = STEER_MIDDLE;
int rc_stop = 0;

// jetson vars
int jetson_throttle = THROTTLE_ZERO;
int jetson_brake = BRAKE_MAX;
int jetson_gear = GEAR_P;
int steer_val = STEER_MIDDLE;
int steer_angle = 0;
int jetson_stop = 0;

void setup()
{
  Serial.begin(9600);   // debug serial
  Serial1.begin(9600);  // Jetson serial
  Serial2.begin(38400); // Small RoboClaw serial (brake and gear shift)
  Serial3.begin(38400); // Large RoboClaw serial (steering)

  // set up pins
  pinMode(Ignition_Relay, OUTPUT);
  pinMode(Battery_Relay, OUTPUT);
  //pinMode(Jetson_Boot, OUTPUT);
  pinMode(Brake_Pot, INPUT);
  pinMode(Gear_Pot, INPUT);
  pinMode(Steering_Pot, INPUT);

  // attach servos
  throttleServo.attach(Throttle_Out);

  // init actuators
  throttleServo.write(THROTTLE_ZERO);
  brake_state = BRAKE_ZERO;
  gear_lever_state = GEAR_P;
  steering_state = STEER_MIDDLE;

  // swtich car power on
  digitalWrite(Battery_Relay, HIGH);
}


void loop()
{
  //setGear(gear_lever_state);
  //setBrake(brake_state);
  setSteering(steering_state);

  // listen for Jetson
  if (Serial.read() == 0xFF)
  {
    // wait for data
    while (!(Serial.available() > 3)); //Have used main serial for keyboard control

    jetson_stop = Serial.read();
    steer_angle = Serial.read();
    jetson_throttle = Serial.read();
    jetson_brake = Serial.read();
    Serial.print("got angle: ");
    Serial.print((int)steer_angle);
    steer_val = STEER_MIN + (float(steer_angle)/120.0) * (STEER_MAX - STEER_MIN);
    Serial.print("got value: ");
    Serial.print((int)steer_val);
    Serial.print('\n');
  }

  // main state machine
  switch (state)
  {
    case PARK:
      {
        // actuate brakes on, throttle as RC
        brake_state = rc_brake;
        throttleServo.write(rc_throttle);
        steering_state = rc_steering;

        // wait for 200 cycles then start the car
        start_delay_counter++;
        if (start_delay_counter > 200) state = START_CAR;

        break;
      }
    case START_CAR:
      {
        // brakes on, throttle off
        brake_state = BRAKE_MAX;
        throttleServo.write(THROTTLE_ZERO);
        steering_state = rc_steering;

        // start engine and put in gear
        //startEngine();
        delay(1000);
        gear_lever_state = GEAR_D;

        state = DRIVE_AI;
        break;
      }
    case DRIVE_AI://keyboard control for now
      {
        brake_state = rc_brake;
        throttleServo.write(rc_throttle);

        //            steeringServo.write(steer_val);
        steering_state = steer_val;

        break;
      }
    default:
      {
        // included for safety, should not ever end up in this state
        // setBrakes(BRAKE_MAX);
        // setThrottle(THROTTLE_ZERO);
        break;
      }
  }

}

void setSteering(int newpos)
{

  //newpos = 600;

  // RoboClaw Serial3 control, channel 2
  // 128 = full reverse
  // 192 = stop
  // 255 = full forward

  int oldpos = analogRead(Steering_Pot);
  //Serial.print("Pot Value");
  //Serial.println(oldpos);

  int error = int(float(abs(newpos - oldpos)) / 5.0);    // crude proportional steering rate control (makes it less twitchy)
  if ( error > 100 ) error = 100;             // then clamp value to max possible steering rate

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

void startEngine()
{
  digitalWrite(Ignition_Relay, HIGH);
  delay(CRANK_TIME);
  digitalWrite(Ignition_Relay, LOW);
}

void stopEngine()
{
  digitalWrite(Battery_Relay, LOW);
  delay(1000);
  digitalWrite(Battery_Relay, HIGH);
}
