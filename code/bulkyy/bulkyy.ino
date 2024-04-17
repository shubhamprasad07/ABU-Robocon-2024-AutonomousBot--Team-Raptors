#include <Servo.h>
Servo myservo;
int pos = 45;
#define MOTOR1_PWM 2  // Front left wheel
#define MOTOR1_DIR 22

#define MOTOR2_PWM 3  // Front right wheel
#define MOTOR2_DIR 23

#define MOTOR3_PWM 4  // Rear left wheel
#define MOTOR3_DIR 24

#define MOTOR4_PWM 5  // Rear right wheel
#define MOTOR4_DIR 25

int mode = 0;

// limit switch
int limitup = 52;
int limitdown = 53;
int limitstatedown;
int limitstateup;

const int stepPin = 9;         // for up down // Stepper motor driver toggles 1,2,4,6 ON
const int dirPin = 8;         // for up down
int speedo = 850;      // for up down

const int stepPinclaw = 7;         // for claw // Stepper motor driver toggles 1,2,4,6 ON
const int dirPinclaw = 27;         // for claw
int speedoclaw = 800;      // for claw

// define ultrasonic and limit sensor pins
const int ultra_echo1 = 6;
const int ultra_echo2 = 13;
const int ultra_trigger1 = 30;
const int ultra_trigger2 = 12;

//variable to store distance
int distance1;
int distance2;
long duration1;
long duration2;

float vy;
float front;
int omega = 0;
int servoAngle = 0;
void calculateWheelSpeeds(float front, float vy, float omega, int speeds[]);
void setMotorSpeeds(int speeds[]);
void stopMotors();

void setup() {

  myservo.write(pos);

  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_DIR, OUTPUT);
  pinMode(MOTOR4_PWM, OUTPUT);
  pinMode(MOTOR4_DIR, OUTPUT);

  pinMode(ultra_echo1, INPUT);
  pinMode(ultra_trigger1, OUTPUT);
  pinMode(ultra_echo2, INPUT);
  pinMode(ultra_trigger2, OUTPUT);

  //pickup, claw
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  pinMode(stepPinclaw, OUTPUT);
  pinMode(dirPinclaw, OUTPUT);


  pinMode(limitup, INPUT_PULLUP);
  pinMode(limitdown, INPUT_PULLUP);

  myservo.attach(11);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {

  // Read data from serial buffer if available
  while (Serial.available() == 0) {
  }
  // Read the incoming bytes until a newline character is received
  String data = Serial.readStringUntil('\r');

  // Extract x, y, and area coordinates from the received string
  int firstCommaIndex = data.indexOf(',');
  int secondCommaIndex = data.indexOf(',', firstCommaIndex + 1);
  int thirdCommaIndex = data.indexOf(',', secondCommaIndex + 1);

  String x_str = data.substring(0, firstCommaIndex);
  String y_str = data.substring(firstCommaIndex + 1, secondCommaIndex);
  String area_str = data.substring(secondCommaIndex + 1);
  String grip = data.substring(thirdCommaIndex + 1);

  // Convert strings to integers
  int x_coordinate = x_str.toInt();
  int y_coordinate = y_str.toInt();
  int area_value = area_str.toInt();
  omega = map(x_coordinate, -320, 320, 50, -50);
  vy = map(y_coordinate, -240, 240, -240, 240);
  front = map(area_value, 43, 0, -50, 50);
  int speeds[4];

  if (vy > 100)
  {
    servoAngle = servoAngle + 1;

    if (servoAngle != 0)
    {
      pos = servoAngle;
    }
    servoAngle = constrain (pos, 28, 45);
    myservo.write(pos);
  }
  else if (vy < 100)
  {
    servoAngle = servoAngle - 1;
    //    servoAngle = constrain (servoAngle, 0, 70);
    if (servoAngle != 0)
    {
      pos = servoAngle;
    }
    servoAngle = constrain (pos, 28, 45);
    myservo.write(pos);
  }

  if (x_coordinate == 0 && y_coordinate == 0 ) {
    calculateWheelSpeeds(0, 0, 40, speeds);
    setMotorSpeeds(speeds);
  }
  else if (20 > omega > -20 && front > 0) {
    calculateWheelSpeeds(50, 0, 0, speeds);
    setMotorSpeeds(speeds);
  }
  else {
    calculateWheelSpeeds(front, vy, omega, speeds);
    setMotorSpeeds(speeds);
  }

  //  Send the processed values back to Raspberry Pi
  //  Serial.print("x-coordinate: ");
  //  Serial.print(x_coordinate);
  //  Serial.print(",y-coordinate: ");
  //  Serial.print(y_coordinate);
  //  Serial.print(", Area value: ");
  //  Serial.println(area_value);
  ultrasonic();
  limitstatedown = digitalRead(limitdown);
  if (limitstatedown == 0) {
    // case 'U'
    digitalWrite(dirPin, LOW); // Set direction for up
    while (digitalRead(limitup) == 1) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(speedo);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(speedo);
    }
  }
  pickupmechanism(area_value);
}

void calculateWheelSpeeds(float front, float vy, float omega, int speeds[]) {
  speeds[0] = (front + omega);   // Front left wheel
  speeds[1] = (front - omega);   // Front right wheel
  speeds[2] = (front - omega);  // Rear left wheel
  speeds[3] = (front + omega);  // Rear right wheel
}

void setMotorSpeeds(int speeds[]) {
  analogWrite(MOTOR1_PWM, abs(speeds[0]));
  digitalWrite(MOTOR1_DIR, speeds[0] < 0 ? HIGH : LOW);

  analogWrite(MOTOR2_PWM, abs(speeds[1]));
  digitalWrite(MOTOR2_DIR, speeds[1] < 0 ? HIGH : LOW);

  analogWrite(MOTOR3_PWM, abs(speeds[2]));
  digitalWrite(MOTOR3_DIR, speeds[2] < 0 ? HIGH : LOW);

  analogWrite(MOTOR4_PWM, abs(speeds[3]));
  digitalWrite(MOTOR4_DIR, speeds[3] < 0 ? HIGH : LOW);
}

void stopMotors() {
  analogWrite(MOTOR1_PWM, 0);
  analogWrite(MOTOR2_PWM, 0);
  analogWrite(MOTOR3_PWM, 0);
  analogWrite(MOTOR4_PWM, 0);
}
void ultrasonic() { // ultrasonic sensor
  //HC-SR04 LOWER
  digitalWrite(ultra_trigger1, LOW);
  delayMicroseconds(2);
  digitalWrite(ultra_trigger1, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultra_trigger1, LOW);

  // Measure the response from the HC-SR04 Echo Pin
  duration1 = pulseIn(ultra_echo1, HIGH);

  //Determine distance from duration1
  //Use 343 metres per second as speed of sound
  distance1 = duration1 * (0.034 / 2);

  //  Serial.print("Distance: ");
  //  Serial.println(distance1);
  //  Serial.print("          ");
  //HC-SR04 UPPER
  digitalWrite(ultra_trigger2, LOW);
  delayMicroseconds(2);
  digitalWrite(ultra_trigger2, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultra_trigger2, LOW);

  // Measure the response from the HC-SR04 Echo Pin
  duration2 = pulseIn(ultra_echo2, HIGH);

  //Determine distance from duration2
  //Use 343 metres per second as speed of sound
  distance2 = duration2 * (0.034 / 2);

  //  Serial.print("Distance: ");
  //  Serial.println(distance2);
}

void pickupmechanism(int area_value) {
  //  Serial.println(area_value);
  if (distance2 < 18 && limitup == 0 && distance1 < 9 )
  {
    //silo
    //case 'O':
   
    digitalWrite(dirPinclaw, LOW); // Set direction for up
    for (int x = 0; x < 300; x++) {
      digitalWrite(stepPinclaw, HIGH);
      delayMicroseconds(speedoclaw);
      digitalWrite(stepPinclaw, LOW);
      delayMicroseconds(speedoclaw);
    }

   

    //    //case 'C':
    //    digitalWrite(dirPinclaw, HIGH); // Set direction for up
    //    for (int x = 0; x < 300; x++) {
    //      digitalWrite(stepPinclaw, HIGH);
    //      delayMicroseconds(speedoclaw);
    //      digitalWrite(stepPinclaw, LOW);
    //      delayMicroseconds(speedoclaw);
    //      stopMotors();
    //    }
    int speeds[4];
    calculateWheelSpeeds(0, 0, 40, speeds);
    setMotorSpeeds(speeds);
    delay(3000);
    Serial.println(0);
  }

  else if (distance1 < 9 && servoAngle < 30 && area_value > 38)
  {
    
    //case 'O':
    digitalWrite(dirPinclaw, LOW); // Set direction for up
    for (int x = 0; x < 300; x++) {
      digitalWrite(stepPinclaw, HIGH);
      delayMicroseconds(speedoclaw);
      digitalWrite(stepPinclaw, LOW);
      delayMicroseconds(speedoclaw);
      
    }
    //case 'D':
    digitalWrite(dirPin, HIGH);
    while (digitalRead(limitdown) == 1) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(speedo);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(speedo);

    }

    //case 'C':
    digitalWrite(dirPinclaw, HIGH); // Set direction for up
    for (int x = 0; x < 300; x++) {
      digitalWrite(stepPinclaw, HIGH);
      delayMicroseconds(speedoclaw);
      digitalWrite(stepPinclaw, LOW);
      delayMicroseconds(speedoclaw);

    }
    // case 'U'
    digitalWrite(dirPin, LOW); // Set direction for up
    while (digitalRead(limitup) == 1) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(speedo);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(speedo);
    }
    int speeds[4];
    calculateWheelSpeeds(0, 0, 40, speeds);
    setMotorSpeeds(speeds);
    delay(3000);

    //    myservo.write(35);
    Serial.println(1);
  }

}
