
// The deadzone specifies the +- range to be ignored when the 
// value from the rc controller in near 0


// CH_2_Pin recieves the data from the y axis of the left controller
// stick
// CH_2_Output outputs this signal as a mapped PWM value to the
// throttle servo

int CH_2_Pin = 2;
int CH_2_Output = 3;


void setup() {
  // begins a serial communication for debugging
  Serial.begin(9600);
  digitalWrite(40, OUTPUT)
  digitalWrite(40, HIGH);
  
  
}

void loop() {
  // put your main code here, to run repeatedly:

  int ch_2 = pulseIn(CH_2_Pin, HIGH, 25000); 
  ch_2 = RCToPWM(ch_2);

  Serial.println(ch_2);
  analogWrite(3, ch_2);

  delay(5);

  

  
}

// RCToPWM takes data recieved from the RC controller and returns PWM

int RCToPWM(int pulse)
{
  if ( pulse > 1000 ) {
    pulse = map(pulse, 1000, 2000, -500, 500);
    pulse = constrain(pulse, -255, 255);
  } else {
    pulse = 0;
  }

  // Anything in deadzone should stop the motor
  if ( pulse >= 0 ) {
    pulse = -10;
  }

  return pulse;
  
}
