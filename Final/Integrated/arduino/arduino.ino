#include <Servo.h>
Servo myservo;

int drive_State = 0;

String data;
int firstCommaIndex;
int secondCommaIndex;
String x_str;
String y_str;
String area_str;

const int servo_pin = 11;
int line_f = 45;
int ball_d = 90;

#define MOTOR1_PWM 2  // Front left wheel
#define MOTOR1_DIR 22

#define MOTOR2_PWM 3  // Front right wheel
#define MOTOR2_DIR 23

#define MOTOR3_PWM 4  // Rear left wheel
#define MOTOR3_DIR 24

#define MOTOR4_PWM 5  // Rear right wheel
#define MOTOR4_DIR 25

int chisu;

int speeds[4];
int speedsSmooth[4];

// limit switch
const int limitup = 53;   //upper: gives 0 when pressed/triggered
const int limitdown = 52; //lower:gives 0 when triggered
int limitstatedown;
int limitstateup;

// Stepper motor for up-down movement
const int stepPin = 9;
const int dirPin = 8;
int speedo = 850;

// Stepper motor for claw mechanism
const int stepPinclaw = 7;
const int dirPinclaw = 27;
int speedoclaw = 800;

// Ultrasonic sensor pins
const int ultra_echo = 6;
const int ultra_echo2 = 13;
const int ultra_trigger = 30;
const int ultra_trigger2 = 12;

int distance = 100;
int distance2 = 100;
volatile long duration;
volatile long duration2;
volatile bool echoReceived1 = false;
volatile bool echoReceived2 = false;
int zhanda = 0;

int x_coordinate;
int y_coordinate;
int area_value;

float vy;
float vx;
float front;
int omega = 0;
int servoAngle = 75;

unsigned long lastUltrasonicRead = 0;
unsigned long ultrasonicInterval = 50; // Adjust this value as needed

void calculateWheelSpeedsObject(float front, float vy, float omega, int speeds[]);
void calculateWheelSpeedsLine(float vx, float omega, int speeds[]);
void setMotorSpeeds(int speeds[]);
void stopMotors();
void startUltrasonicRead();
void pickupMechanism();
void place();

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_DIR, OUTPUT);
  pinMode(MOTOR4_PWM, OUTPUT);
  pinMode(MOTOR4_DIR, OUTPUT);

  pinMode(ultra_echo, INPUT);
  pinMode(ultra_trigger, OUTPUT);
  pinMode(ultra_echo2, INPUT);
  pinMode(ultra_trigger2, OUTPUT);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  pinMode(stepPinclaw, OUTPUT);
  pinMode(dirPinclaw, OUTPUT);

  pinMode(limitup, INPUT_PULLUP);
  pinMode(limitdown, INPUT_PULLUP);

  myservo.attach(servo_pin);
  myservo.write(line_f);

  // Claw up
  digitalWrite(dirPin, LOW);
  while (digitalRead(limitup) == 1) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speedo);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speedo);
  }

  // Claw opening
  digitalWrite(dirPinclaw, LOW); // Set direction
  for (int x = 0; x < 650; x++) {
    digitalWrite(stepPinclaw, HIGH);
    delayMicroseconds(speedo);
    digitalWrite(stepPinclaw, LOW);
    delayMicroseconds(speedo);
  }
}

void loop() {

  if (Serial.available() > 0) {

    if (drive_State == 0) {

      // Read the incoming bytes until a newline character is received
      String data = Serial.readStringUntil('\r');

      // Extract x, y, and area coordinates from the received string
      firstCommaIndex = data.indexOf(',');
      secondCommaIndex = data.indexOf(',', firstCommaIndex + 1);
      x_str = data.substring(0, firstCommaIndex);
      y_str = data.substring(firstCommaIndex + 1, secondCommaIndex);
      area_str = data.substring(secondCommaIndex + 1);

      // Convert strings to integers
      x_coordinate = x_str.toInt();
      y_coordinate = y_str.toInt();
      //int area_value = area_str.toInt();
      omega = map(x_coordinate, -160, 160, -40, 40);
      vy = y_coordinate;
      //  vx = map(area_value, 43, 0, -50, 50);

      vx = 50;
      if (omega > 10 || omega < -10 || vx > 0) {
        //int speeds[4];
        calculateWheelSpeedsLine(vx, omega, speeds);
        setMotorSpeeds(speeds);
      }

      if (x_str == "S") {
        stopMotors();
        myservo.write(ball_d);
        delay(2000);
        drive_State = 1;

        myservo.write(servoAngle);

        limitstatedown = digitalRead(limitdown);
        if (limitstateup == 0) {
          if ( zhanda == 0) {
            //gripper down
            digitalWrite(dirPin, HIGH);
            while (digitalRead(limitdown) == 1) {
              digitalWrite(stepPin, HIGH);
              delayMicroseconds(speedo);
              digitalWrite(stepPin, LOW);
              delayMicroseconds(speedo);
            }
            zhanda = 1;
          }
        }
      }

      if (x_str == "L") {
        vx = 0;
        vy = 0;
        omega = -15.0;
        calculateWheelSpeedsLine(vx, omega, speeds);
        setMotorSpeeds(speeds);
      } else if (x_str == "R") {
        vx = 0;
        vy = 0;
        omega = 15.0;
        calculateWheelSpeedsLine(vx, omega, speeds);
        setMotorSpeeds(speeds);
      } else {
        stopMotors();
      }
    } else if (drive_State == 1) {

      data = Serial.readStringUntil('\r');
      Serial.println(data);
      firstCommaIndex = data.indexOf(',');
      secondCommaIndex = data.indexOf(',', firstCommaIndex + 1);
      x_str = data.substring(0, firstCommaIndex);
      y_str = data.substring(firstCommaIndex + 1, secondCommaIndex);
      area_str = data.substring(secondCommaIndex + 1);

      x_coordinate = x_str.toInt();
      y_coordinate = y_str.toInt();
      area_value = area_str.toInt();
      omega = map(x_coordinate, -550, 550, -55, 55);
      vy = map(y_coordinate, -360, 360, -120, 120);
      front = map(area_value, 135, 1, -30, 70);

      //    Serial.println(servoAngle);

      if (distance < 13 && digitalRead(limitdown) == 0) {
        pickupMechanism();
      } else if (distance2 < 15  && distance < 12 && digitalRead(limitup) == 0 ) {
        if (chisu == 1) {
          place();
        }
      }

      if (servoAngle < 50 || y_coordinate < -80) {
        front = front / 1.8;
        vy = omega / 1.8;
        calculateWheelSpeedsObject(front, vy, 0, speeds);
        setMotorSpeeds(speeds);
      } else if (x_coordinate == 0 && y_coordinate == 0) {
        calculateWheelSpeedsObject(0, 0, 25, speeds);
        setMotorSpeeds(speeds);
      } else if (omega > 8 || omega < -8 ) {
        calculateWheelSpeedsObject(front, 0, omega, speeds);
        setMotorSpeeds(speeds);
      } else if (omega < 8 || omega > -8) {
        calculateWheelSpeedsObject(front, 0, 0, speeds);
        setMotorSpeeds(speeds);
      }

      if (vy > 50) {
        servoAngle = min(servoAngle + 1, 75);
      } else if (vy < -50) {
        servoAngle = max(servoAngle - 1, 15);
      }

      myservo.write(servoAngle);
    }
  }

  // Handle non-blocking ultrasonic sensor readings
  if (millis() - lastUltrasonicRead > ultrasonicInterval) {
    startUltrasonicRead();
    lastUltrasonicRead = millis();
  }
}

void startUltrasonicRead() {
  // Trigger the ultrasonic sensors
  digitalWrite(ultra_trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(ultra_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultra_trigger, LOW);
  duration = pulseIn(ultra_echo, HIGH);
  distance = duration * 0.034 / 2;

  digitalWrite(ultra_trigger2, LOW);
  delayMicroseconds(2);
  digitalWrite(ultra_trigger2, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultra_trigger2, LOW);
  duration2 = pulseIn(ultra_echo2, HIGH);
  distance2 = duration2 * 0.034 / 2;
}

//void echoReceivedISR1() {
//  static unsigned long startTime = 0;
//  if (digitalRead(ultra_echo) == HIGH) {
//    startTime = micros();
//  } else {
//    duration = micros() - startTime;
//    distance = duration * 0.034 / 2;
//    echoReceived1 = true;
//  }
//}
//
//void echoReceivedISR2() {
//  static unsigned long startTime = 0;
//  if (digitalRead(ultra_echo2) == HIGH) {
//    startTime = micros();
//  } else {
//    duration2 = micros() - startTime;
//    distance2 = duration2 * 0.034 / 2;
//    echoReceived2 = true;
//  }
//}

void pickupMechanism() {
  stopMotors();
  // Close the gripper

  digitalWrite(dirPinclaw, HIGH); // Set direction
  for (int x = 0; x < 600; x++) {
    digitalWrite(stepPinclaw, HIGH);
    delayMicroseconds(speedoclaw);
    digitalWrite(stepPinclaw, LOW);
    delayMicroseconds(speedoclaw);
  }

  chisu = 1;
  Serial.println(chisu);

  // Raise the gripper
  digitalWrite(dirPin, LOW); // Set direction
  while (digitalRead(limitup) == 1) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speedo);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speedo);
  }
}

void place() {
  stopMotors();

  // Open the gripper
  digitalWrite(dirPinclaw, LOW); // Set direction
  for (int x = 0; x < 600; x++) {
    digitalWrite(stepPinclaw, HIGH);
    delayMicroseconds(speedo);
    digitalWrite(stepPinclaw, LOW);
    delayMicroseconds(speedo);
  }
  chisu = 0;
  Serial.println(chisu);
}

void calculateWheelSpeedsObject(float front, float vy, float omega, int speeds[]) {
  speeds[0] = (front + vy + omega); // Front left wheel
  speeds[1] = (front - vy - omega); // Front right wheel
  speeds[2] = (front - vy + omega); // Rear left wheel
  speeds[3] = (front + vy - omega); // Rear right wheel

  for (int i = 0; i < 4; i++) {
    speedsSmooth[i] = speedsSmooth[i] + 0.07 * (speeds[i] - speedsSmooth[i]);
  }
}

void calculateWheelSpeedsLine(float vx, float omega, int speeds[]) {
  speeds[0] = (vx + omega );   // Front left wheel
  speeds[1] = (vx - omega );   // Front right wheel
  speeds[2] = (vx + omega );  // Rear left wheel
  speeds[3] = (vx - omega );  // Rear right wheel

  for (int i = 0; i < 4; i++) {
    speedsSmooth[i] = speeds[i];
  }
}

void setMotorSpeeds(int speeds[]) {
  int motorPins[4][2] = {
    {MOTOR1_PWM, MOTOR1_DIR},
    {MOTOR2_PWM, MOTOR2_DIR},
    {MOTOR3_PWM, MOTOR3_DIR},
    {MOTOR4_PWM, MOTOR4_DIR}
  };

  for (int i = 0; i < 4; i++) {
    if (speedsSmooth[i] >= 0) {
      digitalWrite(motorPins[i][1], LOW);
      analogWrite(motorPins[i][0], speedsSmooth[i]);
    } else {
      digitalWrite(motorPins[i][1], HIGH);
      analogWrite(motorPins[i][0], -speedsSmooth[i]);
    }
  }
}

void stopMotors() {
  analogWrite(MOTOR1_PWM, 0);
  analogWrite(MOTOR2_PWM, 0);
  analogWrite(MOTOR3_PWM, 0);
  analogWrite(MOTOR4_PWM, 0);
}
