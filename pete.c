#include <Servo.h>

// states for state machine
#define PARK     0
#define IGNITION 1
#define NEUTRAL_RC  2
#define DRIVE_RC 3
#define DRIVE_AI 4
#define REVERSE  5

// Arduino pins
#define RC_right_y_in   22
#define RC_right_y_out  4
#define RC_right_x_in   5
#define RC_right_x_out  6
#define RC_ignition_switch 10

#define Ignition_Relay  22
#define Battery_Relay   12
#define Jetson_Boot     1111
#define Throttle_Pin    2222
#define Brake_Pin       3333
#define Gear_Pin        4444
#define Steering_Pin    5555

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
void startEngine();
bool setGear(int);
void setBrakes(int);
void setSteering(int);
void setThrottle(int);
void RCToPWM(int);

// init actuators
Servo brakeServo;
Servo throttleServo;
Servo gearServo;
Servo steerServo;

void setup()
{
    Serial.begin(9600);

    // set up pins
    pinMode(Ignition_Relay, OUTPUT);
    pinMode(Battery_Relay, OUTPUT);
    pinMode(Jetson_Boot, OUTPUT);

    // attach servos
    brakeServo.attach(Brake_Pin);
    throttleServo.attach(Throttle_Pin);
    gearServo.attach(Gear_Pin);
    steerServo.attach(Steering_Pin);

    // swtich car power on
    digitalWrite(Battery_Relay, HIGH);

    // boot Jetson
    digitalWrite(Jetson_Boot, HIGH);
    delay(100);
    digitalWrite(Jetson_Boot, LOW);

    // init state machine
    state = PARK;
}

void loop()
{
    // listen to RC controller
    int ch_2_brake = pulseIn(RC_right_y_in, HIGH, 25000);
    ch_2_brake = RCToPWM(ch_2);

    int ch_8_ignition = pulseIn(RC_ignition_switch, HIGH, 25000);
    ch_8_ignition = RCToPWM(ch_8);

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

String parseJetson(String data, char separator, int index)
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

// class Linda
// {
// public:
//     Linda()
//     {
//
//
//
//
//     }
//
//     void Init() {
//
//     }
//
//     void startEngine()
//     {
//         digitalWrite(Ignition_Relay, HIGH);
//         delay(1000);
//         digitalWrite(Ignition_Relay, LOW);
//     }
//
//     void stopEngine()
//     {
//         digitalWrite(Battery_Relay, LOW);
//
//     }
//
//
//     void process_command(int cmd_x_velocity = 0, int cmd_b_velocity = 0, int cmd_theta = 0, int cmd_gamma = 0)
//     {
//         // This is the main function for the RC car control
//         // It decides what action to do based on the current state and command input
//         // RUNS REPEATEDLY, IT MUST BE CALLED FROM THE MAIN LOOP
//
//         // Note: if in RC_TELEOP_STATE, commanded velocities will be ignored, PWM values will be read instead
//
//         lastCommandTimestamp = millis();
//         Serial.println("Processing command");
//
//         // Will be changed into the HALT state if it is not safe to drive.
//         //checkFailsafes();
//
//         int neutral_switch_pos = int(analogRead(NEUTRAL_CONTROL_SWITCH_PIN));
//         int autonomous_switch_pos = int(analogRead(AUTONOMOUS_CONTROL_SWITCH_PIN));
//
//         if(neutral_switch_pos > 750) {
//             if(currentStateID != NEUTRAL_SWITCH_STATE)
//                 set_current_state_ID(NEUTRAL_SWITCH_STATE);
//             //return;
//         } else if(autonomous_switch_pos > 750) {
//             if(currentStateID != AUTONOMOUS_SWITCH_STATE)
//                 set_current_state_ID(AUTONOMOUS_SWITCH_STATE);
//             //return;
//         } else {
//             if(currentStateID != RC_TELEOP_STATE)
//                 set_current_state_ID(RC_TELEOP_STATE);
//             //return;
//         }
//
//         // State Machine
//         switch (currentStateID)
//         {
//             case HALT_STATE:
//                 // We are in HALT_STATE
//
//                 x_velocity = 0.0;
//                 theta = cmd_theta;
//
//                 // Once we have slowed to a HALT, lets stop the engine
//                 if (abs(x_velocity_sensed) <= 0.1)
//                 {
//                     stopEngine();
//                 }
//
//                 // Check the Control State Switch on dash of car
//                 // Neutral - Puts the car gear into neutral
//                 // RC - Allows the car to be driven by Remote Controller
//                 // Autonomous - car is drive by Nvidia Jetson
//
//                 if(neutral_switch_pos > 750) {
//                     set_current_state_ID(NEUTRAL_SWITCH_STATE);
//                     return;
//                 } else if(autonomous_switch_pos > 750) {
//                     set_current_state_ID(AUTONOMOUS_SWITCH_STATE);
//                     return;
//                 } else {
//                     set_current_state_ID(RC_TELEOP_STATE);
//                     // set_current_state_ID(AUTONOMOUS_SWITCH_STATE);
//                     return;
//                 }
//
//                 break;
//
//             case RC_TELEOP_STATE:
//             {
//
//                 break;
//             }
//             case IGNITION_STATE:
//                 // We don't do anything repetedly in ignition state
//                 // (only once: on state change)
//                 break;
//
//             case ENGINE_START_STATE:
//                 // We don't do anything repetedly in ignition state
//                 // (only once: on state change)
//                 break;
//
//             case AUTONOMOUS_SWITCH_STATE:
//                 Serial.println("Autonomous State Selected");
//
//                 // Output the steering angle to the console
//                 Serial.print("#");
//                 Serial.print(double(analogRead(STEERING_ACTUATOR_POSITION_SENSOR_PIN)));
//                 Serial.println("!");
//
//                 //check_ignition_starter();
//
//                 sc.ReadData();
//                 //if(sc.message_time - timeDiff > 500)
//                 if ( sc.message_type != -1 ) {
//                     // to-do: parse the message from the serial
//                 } else {
//                     Serial.println("No command from Jetson received");
//                     return;
//                 }
//
//                 // Send command to the steering controller
//
//                 // Send command to the brake motor controller
//
//                 // Send command to the throttle controller
//
//                 // Send command to the gear controller
//
//                 break;
//         }
//
//     }
//
//     int get_current_gear_position()
//     {
//
//     }
//
//     bool set_current_state_ID(int newStateID)
//     {
//         // This function gets called when a change state is requested
//         // Returns true on a successful transition
//
//         // Code blocks within this switch statement are ONLY CALLED ON STATE change
//         switch (newStateID)
//         {
//             case IGNITION_STATE:
//
//                 // Only allowed to transistion from HALT STATE to IGNITION STATE
//                 // FIXME: add state to MotorController class so that we can request the current and last commanded position
//                 if (currentStateID == HALT_STATE)
//                 {
//                     // Ensure that we are in park before engaging ignition
//                     if (abs(gear_motor->GetCurrentPosition() - PARK_GEAR_POSITION) > GEAR_FEEDBACK_TOLERENCE)
//                     {
//                         Serial.println("Ignition command received, not in park.");
//
//                         //Put the car into park
//                         // current_gear_position = PARK_GEAR_POSITION;
//                         /* DISABLE
//                          gear_motor->SetTargetPosition(current_gear_position);
//                          */
//
//                         return false;
//                     }
//                     else
//                     {
//                         // Once the car is in park, we can start the ignition
//                         Serial.println("Car in park, turning on ignition");
//                         digitalWrite(IGNITION_RELAY_PIN, HIGH);
//                         main_relay_on = 1;
//                         return true;
//                     }
//                     break;
//                 }
//             case ENGINE_START_STATE:
//
//                 // Only transistion to ENGINE_START_STATE if currently in ignition state
//                 if (currentStateID == IGNITION_STATE)
//                 {
//                     startEngine();
//                 }
//                 break;
//             case HALT_STATE:
//                 // Do nothing on transition into HALT
//                 break;
//             case RC_TELEOP_STATE:
//                 // Do nothing on transition into RC_TELEOP
//                 break;
//             case AI_READY_STATE:
//                 // Do nothing on transition into AI
//                 break;
//             case NEUTRAL_SWITCH_STATE:
//                 // Do nothing on transition into NEUTRAL_SWITCH_STATE
//                 break;
//             case AUTONOMOUS_SWITCH_STATE:
//                 // Do nothing on transition into AUTONOMOUS_SWITCH_STATE
//                 break;
//         }
//
//         Serial.print("Changing state to: ");
//         Serial.println(newStateID);
//
//         currentStateID = newStateID;
//         return true;
//     }
//
//     int getcurrentStateID()
//     {
//         // Return the ID of the state that we are currently in
//         return currentStateID;
//     }
//
//
