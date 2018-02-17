#include <Servo.h>

// states for state machine
#define PARK        0
#define IGNITION    1
#define NEUTRAL_RC  2
#define DRIVE_RC    3
#define DRIVE_AI    4
#define REVERSE_RC  5

// Arduino pins
#define RC_ch_2         22 // brake
#define RC_ch_8         24 // ignition

#define gear_pot  A2
#define brake_pot A3

#define Ignition_Relay  13
#define Battery_Relay   12
#define Jetson_Boot     1111
#define Throttle_Out    7
#define Brake_Out       2
#define Gear_Out        3
#define Steering_Out    5555

// gear options
#define GEAR_P 0
#define GEAR_R 1
#define GEAR_N 2
#define GEAR_D 3

// brake constants
#define BRAKE_ZERO 0
#define BRAKE_MAX 180

// throttle constants
#define THROTTLE_ZERO 0
#define THROTTLE_MAX 180

// steering constants
#define STEER_HARD_LEFT     0
#define STEER_MIDDLE        90
#define STEER_HARD_RIGHT    180

// misc constants
#define CRANK_TIME 1000

//function prototypes
//void startEngine();
//bool setGear(int);
//void setBrakes(int);
//void setSteering(int);
//void setThrottle(int);
//void RCToPWM(int);

// init actuators
Servo brakeServo;
Servo throttleServo;
Servo gearServo;
Servo steerServo;

// init state machine
int state = PARK;

void setup()
{
    Serial.begin(9600);

    // set up pins
    pinMode(Ignition_Relay, OUTPUT);
    pinMode(Battery_Relay, OUTPUT);
    pinMode(Jetson_Boot, OUTPUT);

    // attach servos
    brakeServo.attach(Brake_Out);
    throttleServo.attach(Throttle_Out);
    gearServo.attach(Gear_Out);
    steerServo.attach(Steering_Out);

    // swtich car power on
    digitalWrite(Battery_Relay, HIGH);

    // boot Jetson
    digitalWrite(Jetson_Boot, HIGH);
    delay(100);
    digitalWrite(Jetson_Boot, LOW);
}

void loop()
{
    // listen to RC controller
    int ch_2_brake = pulseIn(RC_ch_2, HIGH, 25000);
    ch_2_brake = RCToPWM(ch_2_brake);1111

    int ch_8_ignition = pulseIn(RC_ch_8, HIGH, 25000);
    ch_8_ignition = RCToPWM(ch_8_ignition);

    // listen for Jetson
    int jetson_throttle = 0;
    int jetson_brake = 0;
    int jetson_gear = GEAR_P;
    int jetson_steer = STEER_MIDDLE;

    // maybe not safe for scheduling
    while (Serial.available() > 0) {
        String incomingSignal = Serial.readString();

        int jetson_throttle = parseJetson(incomingSignal,':',0);
        int jetson_brake = parseJetson(incomingSignal,':',1);
        int jetson_steer = parseJetson(incomingSignal,':',2);
        // include stop commant
     }

    // main state machine
    switch(state)
    {
        case PARK:
        {
            // actuate brakes on
            setBrakes(BRAKE_MAX);

            if (ch_8_ignition > 1750)
            {
                state = IGNITION;
            }

            break;
        }
        case IGNITION:
        {
            // brakes on
            setBrakes(BRAKE_MAX);

            // start engine and put in neutral
            startEngine();
            setGear(GEAR_N);

            state = NEUTRAL_RC;
            break;
        }
        case NEUTRAL_RC:
        {
            setBrakes(ch_2_brake);


            // if RC asks for DRIVE_RC
                // set brakes on
                // set throttle zero
                // set gear GEAR_D
                // state = DRIVE_RC

            // if RC asks for DRIVE_AI
                // set brakes on
                // set throttle zero
                // set steering straight
                // set GEAR_D
                // state = DRIVE_AI
            break;
        }
        case DRIVE_RC:
        {
          // listen from controller: throttle, brake, steering, etc

          // set/actuate all outputs

          // // Throttle Control
          // int ch_2 = pulseIn(CH_2_Pin, HIGH, 25000);
          // ch_2 = RCToPWM(ch_2);
          // analogWrite(RC_right_y_out, 255 - ch_2);

            break;
        }
        case DRIVE_AI:
        {
            // listen for Jetson command
            // set/actuate all outputs
            break;
        }
        case REVERSE_RC:
        {

            break;
        }
        default:
        {
            // should not ever end up in this state
            setBrakes(BRAKE_MAX);
            setThrottle(THROTTLE_ZERO);
            break;
        }
    }

}

void startEngine()
{
    digitalWrite(Ignition_Relay, HIGH);
    delay(CRANK_TIME);
    digitalWrite(Ignition_Relay, LOW);
}

bool setGear(int gear)
{
    // set gear position (actuate shifter)

    //ONLY RETURN ONCE GEAR IS IN POSITION
}

void setBrakes(int brake_pos)
{

}

void setSteering(int steer_angle)
{

}

void setThrottle(int throttle_pos)
{

}

int RCToPWM(int pulse)
{
  if ( pulse > 1000 ) {
    pulse = map(pulse, 1000, 2000, -500, 500);
    pulse = constrain(pulse, -255, 255);
  } else {
    pulse = 0;
  }

  // Anything in deadzone should stop the motor

  return pulse;

}

int parseJetson(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    String val_string = found > index ? data.substring(strIndex[0], strIndex[1]) : "";
    int val_integer = val_string.toInt();

    return val_integer;
}
