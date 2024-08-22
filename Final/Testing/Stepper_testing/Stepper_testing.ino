// limit switch
const int limitup = 53;
const int limitdown = 52;

// Stepper motor for up-down movement
const int stepPin = 9;
const int dirPin = 8;
int speedo = 500;

// Stepper motor for claw mechanism
const int stepPinclaw = 7;
const int dirPinclaw = 27;
int speedoclaw = 800;

void setup() {
  pinMode(limitup, INPUT_PULLUP);
  pinMode(limitdown, INPUT_PULLUP);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPinclaw, OUTPUT);
  pinMode(dirPinclaw, OUTPUT);
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  if (Serial.available() > 0) {
    char input_value = Serial.read(); // Read an integer value from serial monitor
    Serial.println(input_value);
    if (input_value == 'C') { // Check if the input value is C
      digitalWrite(13, HIGH); // Indicate motor movement

      digitalWrite(dirPinclaw, HIGH); // Set direction
      for (int x = 0; x < 800; x++) {
        digitalWrite(stepPinclaw, HIGH);
        delayMicroseconds(speedo);
        digitalWrite(stepPinclaw, LOW);
        delayMicroseconds(speedo);
      }
    }
    if (input_value == 'O') { // Check if the input value is O
      digitalWrite(13, LOW); // Indicate motor movement

      digitalWrite(dirPinclaw, LOW); // Set direction
      for (int x = 0; x < 800; x++) {
        digitalWrite(stepPinclaw, HIGH);
        delayMicroseconds(speedo);
        digitalWrite(stepPinclaw, LOW);
        delayMicroseconds(speedo);
      }
    }
    if (input_value == 'D'){
      digitalWrite(dirPin, HIGH);
      while (digitalRead(limitdown) == 1)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(speedo);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(speedo);
      }
    }
    if (input_value == 'U'){
      digitalWrite(dirPin, LOW);
      while (digitalRead(limitup) == 1)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(speedo);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(speedo);
      }
    }
  }
}
