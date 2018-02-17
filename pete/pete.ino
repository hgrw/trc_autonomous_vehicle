#include <Servo.h>

// states for state machine
#define PARK        0
#define IGNITION    1
#define NEUTRAL_RC  2
#define DRIVE_RC    3
#define DRIVE_AI    4
#define REVERSE_RC  5

// Arduino pins
#define RC_CH_2     22 // brake (left stick, y axis)
#define RC_CH_8     26 // ignition (left bottom trigger toggle)
#define RC_CH_1     3  // steering (right stick, x axis)
#define RC_CH_7     28 // test input for ignition

#define Gear_Pot  A2
#define Brake_Pot A3

#define Ignition_Relay  9
#define Battery_Relay   8
#define Jetson_Boot     0 // fix this
#define Throttle_Out    7
#define Brake_Out       5
#define Gear_Out        0 // fix this
#define Steering_Out    4

// gear options
#define GEAR_P 0
#define GEAR_R 1
#define GEAR_N 2
#define GEAR_D 3

// brake constants
#define BRAKE_ZERO  0
#define BRAKE_MAX   180

// throttle constants
#define THROTTLE_ZERO 180
#define THROTTLE_MAX  90

// steering constants
#define STEER_HARD_LEFT     0
#define STEER_MIDDLE        90
#define STEER_HARD_RIGHT    180

// misc constants
#define CRANK_TIME 1000

// init actuators
Servo brakeServo;
Servo throttleServo;
Servo gearServo;
Servo steeringServo;

// init state machine
int state = PARK;

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
int jetson_steer = STEER_MIDDLE;
int jetson_stop = 0;

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
    steeringServo.attach(Steering_Out);

    // init servos
    brakeServo.write(BRAKE_MAX);
    throttleServo.write(THROTTLE_ZERO);
    gearServo.write(GEAR_P);
    steeringServo.write(STEER_MIDDLE);

    // swtich car power on
    digitalWrite(Battery_Relay, HIGH);

    // boot Jetson
    digitalWrite(Jetson_Boot, HIGH);
    delay(100);
    digitalWrite(Jetson_Boot, LOW);
}

void loop()
{
    // left stick from RC controller
    int ch_2_stick = pulseIn(RC_CH_2, HIGH, 25000);
    int ch_2_PWM = RCToThrottle(ch_2_stick);
    
    // stick down, brakes on
    if (ch_2_PWM > 100)
    {
        rc_throttle = THROTTLE_ZERO;
        rc_brake = BRAKE_MAX;
    }
    // stick up, throttle on
    else if (ch_2_PWM < -100)
    {
        rc_throttle = THROTTLE_MAX;
        rc_brake = BRAKE_ZERO;
    }
    // not braking or throttle
    else if (ch_2_PWM < 100 && ch_2_PWM > -100)
    {
        rc_throttle = THROTTLE_ZERO;
        rc_brake = BRAKE_ZERO;
    }

    // igntion from RC controller
    int ch_8_ignition = pulseIn(RC_CH_8, HIGH, 25000);
//    Serial.print(ch_8_ignition);
//    Serial.print("\t");
//    Serial.println(rc_throttle);
    bool ignition_flag = false;    

    if (ch_8_ignition > 1750) ignition_flag = true;
    else ignition_flag = false;

    int ch_1_steering = pulseIn(RC_CH_1, HIGH, 25000);
    int ch_1_PWM = RCToSteering(ch_1_steering);
    
    Serial.print("steering: ");
    Serial.print(ch_1_PWM);
    Serial.print("\t");
    Serial.println(ch_1_steering);
    
//    rc_steering = ch_1_PWM;
    steeringServo.write(ch_1_PWM);

    // delay for testing steering
    delay(10);

    // listen for Jetson
    // maybe not safe for scheduling
    while (Serial.available() > 0) {
        String incomingSignal = Serial.readString();

        int jetson_throttle = parseJetson(incomingSignal,':',0);
        int jetson_brake = parseJetson(incomingSignal,':',1);
        int jetson_steer = parseJetson(incomingSignal,':',2);
        int jetson_gear = parseJetson(incomingSignal,':',3);
        int jetson_stop = parseJetson(incomingSignal,':',4);
    }

    // main state machine
    switch(state)
    {
        case PARK:
        {
            // actuate brakes on, throttle as RC
            brakeServo.write(BRAKE_MAX);
            throttleServo.write(rc_throttle);

            if (ignition_flag)
            {
                state = IGNITION;
            }

            break;
        }
        case IGNITION:
        {
            // reset flag
            ignition_flag = false;
          
            // brakes on, throttle off
            brakeServo.write(BRAKE_MAX);
            throttleServo.write(THROTTLE_ZERO);

            // start engine and put in neutral
            startEngine();
            setGear(GEAR_N);

            state = DRIVE_RC;
            break;
        }
        case NEUTRAL_RC:
        {
            brakeServo.write(rc_brake);
            throttleServo.write(rc_throttle);

            // if RC asks for DRIVE_RC
                // setBrakes(BRAKE_ZERO);
                // setThrottle(THROTTLE_ZERO);
                // setGear(GEAR_D);
                // state = DRIVE_RC;

            // if RC asks for DRIVE_AI
                // setBrakes(BRAKE_ZERO);
                // setThrottle(THROTTLE_ZERO);
                // setSteering(STEER_MIDDLE);
                // setGear(GEAR_D);
                // state = DRIVE_AI;
            break;
        }
        case DRIVE_RC:
        {
            // set/actuate all outputs
            brakeServo.write(rc_brake);
            throttleServo.write(rc_throttle);
            steeringServo.write(rc_steering);

            // if request AI transition
                // setBrakes(BRAKE_ZERO);
                // setThrottle(THROTTLE_ZERO);
                // setSteering(STEER_MIDDLE);
                // setGear(GEAR_D);
                // state = DRIVE_AI;
            break;
        }
        case DRIVE_AI:
        {
            // if stop command, switch off and go back to park mode
            if (jetson_stop == 1)
            {
                stopEngine();
                state = PARK;
                break;
            }

            //setThrottle(jetson_throttle);
            //setBrakes(jetson_brake);
            //setSteering(jetson_steer);
            setGear(jetson_gear);
            break;
        }
        case REVERSE_RC:
        {

            break;
        }
        default:
        {
            // should not ever end up in this state
            // setBrakes(BRAKE_MAX);
            // setThrottle(THROTTLE_ZERO);
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

void stopEngine()
{
    digitalWrite(Battery_Relay, LOW);
    delay(1000);
    digitalWrite(Battery_Relay, HIGH);
}

bool setGear(int gear)
{
    // set gear position (actuate shifter)

    //ONLY RETURN ONCE GEAR IS IN POSITION
}

int RCToThrottle(int pulse)
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

int RCToSteering(int pulse)
{
    if ( pulse > 1000 ) {
      pulse = map(pulse, 1110, 1930, 0, 180);
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
