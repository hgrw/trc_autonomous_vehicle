#include <Servo.h>

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
//#define RC_CH_7     2 // ignition (left bottom trigger toggle)
#define RC_CH_1     3 // steering (right stick, x axis)

#define Gear_Pot  A7
#define Brake_Pot A0

#define Ignition_Relay  9
#define Battery_Relay   8
#define Jetson_Boot     0 // fix this
#define Throttle_Out    7
#define Brake_Dir_Out   34
#define Brake_PWM_Out   36
#define Gear_Dir_Out    52
#define Gear_PWM_Out    48
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
#define THROTTLE_ZERO     170
#define THROTTLE_LOW      160
#define THROTTLE_SENSIBLE 120
#define THROTTLE_MAX      90

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

// misc vars
int gear_lever_state = GEAR_P;
int brake_state = BRAKE_ZERO;
int delay_counter = 0;
int dead_man_counter = 0;
int prev_stick = 0;

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
    Serial1.begin(9600);

    // set up pins
    pinMode(Ignition_Relay, OUTPUT);
    pinMode(Battery_Relay, OUTPUT);
    pinMode(Jetson_Boot, OUTPUT);
    pinMode(Gear_PWM_Out, OUTPUT);
    pinMode(Gear_Dir_Out, OUTPUT);

    // attach servos
    throttleServo.attach(Throttle_Out);
    gearServo.attach(Gear_PWM_Out);
    steeringServo.attach(Steering_Out);
    pinMode(Brake_PWM_Out, OUTPUT);
    pinMode(Brake_Dir_Out, OUTPUT);
    
    // init servos
    brake_state = BRAKE_ZERO;
    throttleServo.write(THROTTLE_ZERO);
    steeringServo.write(STEER_MIDDLE);
    gear_lever_state = GEAR_P;

    // swtich car power on
    digitalWrite(Battery_Relay, HIGH);

    // boot Jetson
    digitalWrite(Jetson_Boot, HIGH);
    delay(100);
    digitalWrite(Jetson_Boot, LOW);
}


void loop()
{  
    delay(5);

    steeringServo.write(45);

    delay(10000);
  
    // left stick from RC controller
    int ch_2_stick = pulseIn(RC_CH_2, HIGH, 25000);
    int ch_2_PWM = RCToThrottle(ch_2_stick);

    if (ch_2_stick == prev_stick) dead_man_counter++;
    else dead_man_counter = 0;
    
    // stick down, brakes on
    if (ch_2_PWM > 100)
    {
        rc_throttle = THROTTLE_ZERO;
        rc_brake = map(ch_2_PWM, 101, 255, 0, 1023);
    }
    // stick up, throttle on
    else if (ch_2_PWM < -100)
    {
        rc_throttle = THROTTLE_SENSIBLE;
        rc_brake = BRAKE_ZERO;
    }
    // not braking or throttle
    else if (ch_2_PWM < 100 && ch_2_PWM > -100)
    {
        rc_throttle = THROTTLE_ZERO;
        rc_brake = BRAKE_ZERO;
    }

// CAN'T READ MORE THAN TWO PWM AT ONCE
//    // igntion from RC controller
//    int ch_3_ignition = pulseIn(RC_CH_3, HIGH, 25000);
//    Serial.print(ch_3_ignition);
//    Serial.print("\t");
//    Serial.println(rc_throttle);
//    bool ignition_flag = false;    
//
//    if (ch_3_ignition > 1750) ignition_flag = true;
//    else ignition_flag = false;

    int ch_1_steering = pulseIn(RC_CH_1, HIGH, 25000);
    int ch_1_PWM = RCToSteering(ch_1_steering);
    
//    Serial.print("steering: ");
//    Serial.print(ch_1_PWM);
//    Serial.print("\t");
//    Serial.println(ch_1_steering);
    
    rc_steering = ch_1_PWM;

    // gear and brake loops select
    setGear(gear_lever_state);
    setBrake(brake_state);

    // listen for Jetson
    if (Serial1.read() == 0xFF)
    {
        Serial.println("buffer");
        
        // wait for data
        while (!(Serial1.available() > 3)) Serial.println("wait");
        
        jetson_stop = Serial1.read();
        jetson_steer = Serial1.read();
        jetson_throttle = Serial1.read();
        jetson_brake = Serial1.read();

//        Serial.print(" stop: ");
//        Serial.print(jetson_stop);
//        Serial.print(" steer: ");
//        Serial.print(jetson_steer);
//        Serial.print(" jetson_throttle: ");
//        Serial.print(jetson_throttle);
//        Serial.print(" jetson_brake: ");
//        Serial.println(jetson_brake);
    }
    
//    // maybe not safe for scheduling
//    while (Serial1.available()) 
//    {
//        String incomingSignal = Serial1.readString();
//
//        int jetson_throttle = parseJetson(incomingSignal,':',0);
//        int jetson_brake = parseJetson(incomingSignal,':',1);
//        int jetson_steer = parseJetson(incomingSignal,':',2);
//        int jetson_gear = parseJetson(incomingSignal,':',3);
//        int jetson_stop = parseJetson(incomingSignal,':',4);
//
//        Serial.print("rec\t");
//        Serial.println(incomingSignal);
//    }

    // main state machine
    switch(state)
    {
        case PARK:
        {
            // actuate brakes on, throttle as RC
            brake_state = rc_brake;
            throttleServo.write(rc_throttle);
            steeringServo.write(STEER_MIDDLE);

//            Serial.print("brake: ");
//            Serial.print(rc_brake);
//            Serial.print("\tsteering: ");
//            Serial.println(rc_steering);

            delay_counter++;
            if (delay_counter > 200) state = START_CAR;

            break;
        }
        case START_CAR:
        {          
            // brakes on, throttle off
            brake_state = BRAKE_MAX;
            throttleServo.write(THROTTLE_ZERO);
            steeringServo.write(STEER_MIDDLE);

            // start engine and put in gear
            startEngine();
            delay(1000);
            gear_lever_state = GEAR_D;
            
            state = DRIVE_AI;
            break;
        }
//        case NEUTRAL_RC:
//        {
//            setBrake.write(rc_brake);
//            throttleServo.write(rc_throttle);
//
//            // if RC asks for DRIVE_RC
//                // setBrakes(BRAKE_ZERO);
//                // setThrottle(THROTTLE_ZERO);
//                // gear_lever_state = (GEAR_D);
//                // state = DRIVE_RC;
//
//            // if RC asks for DRIVE_AI
//                // setBrakes(BRAKE_ZERO);
//                // setThrottle(THROTTLE_ZERO);
//                // setSteering(STEER_MIDDLE);
//                // gear_lever_state = (GEAR_D);
//                // state = DRIVE_AI;
//            break;
//        }
        case DRIVE_RC:
        {
            // set/actuate all outputs
            brake_state = rc_brake;
            throttleServo.write(rc_throttle);
            steeringServo.write(rc_steering);

//            if (ignition_flag)
//            {
//                ignition_flag = false;
//                state = STOP_CAR;
//            }

            // if request AI transition
                // setBrakes(BRAKE_ZERO);
                // setThrottle(THROTTLE_ZERO);
                // setSteering(STEER_MIDDLE);
                // gear_lever_state = (GEAR_D);
                // state = DRIVE_AI;
            break;
        }
        case DRIVE_AI:
        {
            // if stop command, switch off and go back to park mode
//            if (jetson_stop == 1)
//            {
//                state = STOP_CAR;
//                break;
//            }

            // set/actuate all outputs
            if (jetson_brake > 0) brake_state = BRAKE_MAX;
            else brake_state = BRAKE_ZERO;

            if (jetson_throttle > 0) throttleServo.write(THROTTLE_LOW);
            else throttleServo.write(THROTTLE_ZERO);
            
            steeringServo.write(jetson_steer);
                          
            break;
        }
//        case REVERSE_RC:
//        {
//
//            break;
//        }
//        case STOP_CAR:
//        {
//            brake_state = BRAKE_MAX;
//            
//            stopEngine();
//            delay(500);
//            gear_lever_state = PARK;
//
//            state = PARK;
//            break;
//        }
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

//void setBrake(int brake_val)
//{
//    moveBrakeActuator(brake_val);
//}

void setBrake(int newpos)
{
    int oldpos = analogRead(Brake_Pot);
    
    if (oldpos >= newpos) 
    {
        digitalWrite(Brake_Dir_Out, HIGH);
        analogWrite(Brake_PWM_Out, 1023);

    } 
    else if (newpos > oldpos) 
    {
        digitalWrite(Brake_Dir_Out, LOW);
        analogWrite(Brake_PWM_Out, 1023);
    }

}

void setGear(int gearState)
{
    switch (gearState) 
    {
        case GEAR_P:
            //Parking
            moveGearActuator(700);
            break;
        case GEAR_R:
            //Reverse
            //moveGearActuator(535);
            break;
        case GEAR_N:
            //Neutral
            //moveGearActuator(477);
            break;
        case GEAR_D:
            //Drive
            moveGearActuator(400);
        default:
            // statements
            return;
     }
}

void moveGearActuator(int newpos)
{
    int oldpos = analogRead(Gear_Pot);
  
    if (oldpos >= newpos )
    {
        gearServo.write(180);
        digitalWrite(Gear_Dir_Out, HIGH);
    } 
    else if (newpos > oldpos) 
    {
        digitalWrite(Gear_Dir_Out, LOW);
        gearServo.write(180);
    }
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
