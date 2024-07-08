#include <Servo.h>
Servo myservo;

int drive_State = 0;

String data;
int firstCommaIndex;
int secondCommaIndex;
String x_str;
String y_str;
String area_str;


int servo_pin {11};
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
long duration;
long duration2;
int zhanda = 0;

int x_coordinate;
int y_coordinate;
int area_value;

float vy;
float vx;
float front;
int omega = 0;
int servoAngle = 75;

void calculateWheelSpeedsObject(float front, float vy, float omega, int speeds[]);
void calculateWheelSpeedsLine(float vx, float omega, int speeds[]);
void setMotorSpeeds(int speeds[]);
void stopMotors();
void ultrasonic();
void pickupMechanism();
void place();

void setup() {

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
  while (digitalRead(limitup) == 1)
  {
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

  // Initialize serial communication
  Serial.begin(9600);
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
          if ( zhanda == 0)
          {
            //gripper down
            digitalWrite(dirPin, HIGH);
            while (digitalRead(limitdown) == 1)
            {
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
      }

      else if (x_str == "R") {
        vx = 0;
        vy = 0;
        omega = 15.0;
        calculateWheelSpeedsLine(vx, omega, speeds);
        setMotorSpeeds(speeds);
      }

      else {
        //stopMotors();

      }
    }

    else if (drive_State == 1)
    {

      data = Serial.readStringUntil('\r');

      firstCommaIndex = data.indexOf(',');
      secondCommaIndex = data.indexOf(',', firstCommaIndex + 1);
      x_str = data.substring(0, firstCommaIndex);
      y_str = data.substring(firstCommaIndex + 1, secondCommaIndex);
      area_str = data.substring(secondCommaIndex + 1);

      if (x_str == 'S')
      {
        stopMotors();
      }

      x_coordinate = x_str.toInt();
      y_coordinate = y_str.toInt();
      area_value = area_str.toInt();
      omega = map(x_coordinate, -550, 550, -55, 55);
      vy = map(y_coordinate, -360, 360, -120, 120);
      front = map(area_value, 135, 1, -30, 70);

      //    Serial.println(servoAngle);

      if (servoAngle < 50 || y_coordinate < -80)
      {
        front = front / 1.8;
        vy = omega / 1.8;
        calculateWheelSpeedsObject(front, vy, 0, speeds);
        setMotorSpeeds(speeds);
      }
      //
      //      Serial.print(omega);
      //      Serial.print(" , ");
      //      Serial.print(vy);
      //      Serial.print(" , ");
      //      Serial.println(front);

      if (distance < 13 && digitalRead(limitdown) == 0)
      {
        pickupMechanism();
      }
      else if (distance2 < 15  && distance < 12 && digitalRead(limitup) == 0 ) {
        if (chisu == 1); {
          place();
        }
      }

      if (x_coordinate == 0 && y_coordinate == 0) {
        calculateWheelSpeedsObject(0, 0, 25, speeds);
        setMotorSpeeds(speeds);
      }
      else if (omega > 8 || omega < -8 ) {
        calculateWheelSpeedsObject(front, 0, omega, speeds);
        setMotorSpeeds(speeds);
      }
      else if (omega < 8 || omega > -8) {
        calculateWheelSpeedsObject(front, 0, 0, speeds);
        setMotorSpeeds(speeds);
      }
      else {
        stopMotors();
      }

      if (vy > 50) {
        servoAngle = min(servoAngle + 2, 75);
      }
      else if (vy < -50) {
        servoAngle = max(servoAngle - 2, 15);
      }
      myservo.write(servoAngle);
      ultrasonic();
    }
  }
}

void calculateWheelSpeedsLine(float vx, float omega, int speeds[]) {
  speeds[0] = (vx + omega );   // Front left wheel
  speeds[1] = (vx - omega );   // Front right wheel
  speeds[2] = (vx + omega );  // Rear left wheel
  speeds[3] = (vx - omega );  // Rear right wheel
}

void calculateWheelSpeedsObject(float front, float vy, float omega, int speeds[]) {
  speeds[0] = (front + vy + omega); // Front left wheel
  speeds[1] = (front - vy - omega); // Front right wheel
  speeds[2] = (front - vy + omega); // Rear left wheel
  speeds[3] = (front + vy - omega); // Rear right wheel

  for (int i = 0; i < 4; i++) {
    speeds[i] = constrain(speeds[i], -100, 100);
  }
}

void setMotorSpeeds(int speeds[])
{
  const float smoothingFactor = 0.9; // Adjust this value as needed for smoother movement
  for (int i = 0; i < 4; i++)
  {
    speedsSmooth[i] = smoothingFactor * speeds[i] + (1 - smoothingFactor) * speedsSmooth[i];
  }

  analogWrite(MOTOR1_PWM, abs(speedsSmooth[0]));
  digitalWrite(MOTOR1_DIR, speedsSmooth[0] < 0 ? HIGH : LOW);

  analogWrite(MOTOR2_PWM, abs(speedsSmooth[1]));
  digitalWrite(MOTOR2_DIR, speedsSmooth[1] < 0 ? HIGH : LOW);

  analogWrite(MOTOR3_PWM, abs(speedsSmooth[2]));
  digitalWrite(MOTOR3_DIR, speedsSmooth[2] < 0 ? HIGH : LOW);

  analogWrite(MOTOR4_PWM, abs(speedsSmooth[3]));
  digitalWrite(MOTOR4_DIR, speedsSmooth[3] < 0 ? HIGH : LOW);
}

void stopMotors() {
  analogWrite(MOTOR1_PWM, 0);
  analogWrite(MOTOR2_PWM, 0);
  analogWrite(MOTOR3_PWM, 0);
  analogWrite(MOTOR4_PWM, 0);
}

void ultrasonic()
{
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

  //  Serial.println(distance2);
}

void pickupMechanism() {
  stopMotors();

  ////gripper close
  digitalWrite(dirPinclaw, HIGH);
  for (int x = 0; x < 600; x++)
  {
    digitalWrite(stepPinclaw, HIGH);
    delayMicroseconds(speedoclaw);
    digitalWrite(stepPinclaw, LOW);
    delayMicroseconds(speedoclaw);
  }
  //gripper up
  digitalWrite(dirPin, LOW);
  while (digitalRead(limitup) == 1)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speedo);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speedo);
  }
  ultrasonic();
  if (distance2 > 20)
  {
    ////gripper open
    digitalWrite(dirPinclaw, LOW);
    for (int x = 0; x < 600; x++)
    {
      digitalWrite(stepPinclaw, HIGH);
      delayMicroseconds(speedoclaw);
      digitalWrite(stepPinclaw, LOW);
      delayMicroseconds(speedoclaw);
    }
    //gripper down
    digitalWrite(dirPin, HIGH);
    while (digitalRead(limitdown) == 1)
    {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(speedo);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(speedo);
    }
    chisu = 0;
    Serial.println(chisu);

  }
  else if (distance2 < 15) {
    servoAngle = 90;
    myservo.write(servoAngle);
    chisu = 1;
    Serial.println(chisu);
  }
}

void place() {
  stopMotors();
  //gripper open
  digitalWrite(dirPinclaw, LOW);

  for (int x = 0; x < 700; x++)
  {
    digitalWrite(stepPinclaw, HIGH);
    delayMicroseconds(speedoclaw);
    digitalWrite(stepPinclaw, LOW);
    delayMicroseconds(speedoclaw);
  }

  for ( int i = 0 ; i < 20000 ; i++)
  {
    calculateWheelSpeedsObject(-45, 0, 0, speeds);
    setMotorSpeeds(speeds);
  }

  for ( int i = 0 ; i < 15000 ; i++)
  {
    calculateWheelSpeedsObject(0, 0, 40, speeds);
    setMotorSpeeds(speeds);
  }
  //  gripper down
  digitalWrite(dirPin, HIGH);
  while (digitalRead(limitdown) == 1)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speedo);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speedo);
  }
  if (chisu != 0)
  { // Check if chisu has changed
    chisu = 0;
    Serial.println(chisu);
    myservo.write(75);
  }
}
