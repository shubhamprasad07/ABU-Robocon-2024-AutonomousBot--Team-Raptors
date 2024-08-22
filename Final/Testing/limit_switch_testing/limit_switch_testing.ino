const int limitdown = 52;
const int limitup = 53;

int state1; // state for limitup
int state2; // state for limitdown

void setup() {
  Serial.begin(9600);
  pinMode(limitdown, INPUT_PULLUP);
  pinMode(limitup, INPUT_PULLUP);
  
}

void loop() {
  state1 = digitalRead(limitup);
  state2 = digitalRead(limitdown);
  Serial.print("Limitup: ");
  Serial.print(state1);
  Serial.print(" || ");
  Serial.print("Limitdown: ");
  Serial.println(state2);
}
