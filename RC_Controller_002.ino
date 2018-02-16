
// The deadzone specifies the +- range to be ignored when the 
// value from the rc controller in near 0
int deadzone = 50;

// CH_2_Pin recieves the data from the y axis of the left controller
// stick
// CH_2_Output outputs this signal as a mapped PWM value to the
// throttle servo

// int CH_1_Pin LEFT AND RIGHT, right stick
// int CH_3_pin Up and down, right stick
// int CH_4_pin Left Right, left stick

int CH_2_Pin = 3;
int CH_2_Output = 4;

// Throttle Start Pin

int Battery_Active_Pin = 12;

int CH_8_Pin = 10;
int CH_8_Output = 11;


void setup() {
  // begins a serial communication for debugging
  Serial.begin(9600);

  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  
  
}

void loop() {
  // put your main code here, to run repeatedly:

  int ch_2 = pulseIn(CH_2_Pin, HIGH, 25000); 
  
  ch_2 = RCToPWM(ch_2);

  analogWrite(3, 255 - ch_2);

  int ch_8 = pulseIn(CH_8_Pin, HIGH, 25000);


  if (ch_8 > 1750)
  {
      digitalWrite(CH_8_Output, HIGH);
  } else 
  {
      digitalWrite(CH_8_Output, LOW);  
  }
  
  Serial.println(ch_8);
  
  delay(5);

  

  
}

// RCToPWM takes data recieved from the RC controller and returns PWM

int engineStart()
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
  if ( pulse <= deadzone ) {
    pulse = 0;
  }

  return pulse;
  
}
