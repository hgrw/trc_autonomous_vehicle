int relay_pin = 9;
int engine_pin = 8;


void setup() {
  
  pinMode(relay_pin, OUTPUT);
  pinMode(engine_pin, OUTPUT);

}

void loop()
{
  digitalWrite(relay_pin, HIGH);
  delay(5000);
  startEngine(); 
  delay(1000000); 
}

void startEngine()
{

  digitalWrite(engine_pin, HIGH);
  delay(1000);
  digitalWrite(engine_pin, LOW);
  
}
