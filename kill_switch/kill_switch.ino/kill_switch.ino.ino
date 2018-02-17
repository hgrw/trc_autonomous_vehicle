#define RC_IN     3
#define RELAY_OUT 8

void setup() {
    Serial.begin(9600);
    pinMode(RELAY_OUT, OUTPUT);
    digitalWrite(RELAY_OUT, HIGH);
}

void loop() {
    // igntion from RC controller
    int rc_input = pulseIn(RC_IN, HIGH, 25000);    
    delay(5);
    
    if (rc_input > 1750) digitalWrite(RELAY_OUT, LOW);
    Serial.println(rc_input);
}
