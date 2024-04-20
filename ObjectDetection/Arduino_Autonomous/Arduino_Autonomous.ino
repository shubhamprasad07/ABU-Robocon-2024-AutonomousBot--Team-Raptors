#include <Servo.h>
Servo myservo;

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
const int limitup = 53;
const int limitdown = 52;
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
int distance;
int distance2;
long duration;
long duration2;

float vy;
float front;
int omega = 0;
int servoAngle = 30;

void calculateWheelSpeeds(float front, float vy, float omega, int speeds[]);
void setMotorSpeeds(int speeds[]);
void stopMotors();
void ultrasonic();
void pickupMechanism();
void place();

///////////////////////////////////////////////////////////////////////////////////////////////
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

  myservo.attach(11);
  myservo.write(servoAngle);
  Serial.begin(9600);

  // Initialize smoothed speeds to 0
  for (int i = 0; i < 4; i++) {
    speedsSmooth[i] = 0;
  }
}

////////////////////////////////////////////////////////////////
void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\r');

    int firstCommaIndex = data.indexOf(',');
    int secondCommaIndex = data.indexOf(',', firstCommaIndex + 1);
    String x_str = data.substring(0, firstCommaIndex);
    String y_str = data.substring(firstCommaIndex + 1, secondCommaIndex);
    String area_str = data.substring(secondCommaIndex + 1);


    int x_coordinate = x_str.toInt();
    int y_coordinate = y_str.toInt();
    int area_value = area_str.toInt();
    omega = map(x_coordinate, -320, 320, -50, 50);
    vy = map(y_coordinate, -240, 240, -120, 120);
    front = map(area_value, 160, 20, -30, 60);

    //    Serial.println(servoAngle);

    if (servoAngle < 15 || y_coordinate < -170)
    {
      front = front / 2;
      vy = omega / 2;
      calculateWheelSpeeds(front, vy, 0, speeds);
      setMotorSpeeds(speeds);
    }

    //    Serial.print(omega);
    //    Serial.print(" , ");
    //    Serial.print(vy);
    //    Serial.print(" , ");
    //    Serial.println(front);

    if (x_coordinate == 0 && y_coordinate == 0) {
      calculateWheelSpeeds(0, 0, 20, speeds);
      setMotorSpeeds(speeds);
    } else if (omega > 10 || omega < -10 ) {
      calculateWheelSpeeds(front, 0, omega, speeds);
      setMotorSpeeds(speeds);
    } else if (omega < 10 || omega > -10 && front > 0) {
      calculateWheelSpeeds(front, 0, 0, speeds);
      setMotorSpeeds(speeds);
    } else {
      stopMotors();
    }
    if (vy > 30) {
      servoAngle = min(servoAngle + 1, 50);
    } else if (vy < -30) {
      servoAngle = max(servoAngle - 1, 2);
    }
    myservo.write(servoAngle);
    ultrasonic();

    limitstatedown = digitalRead(limitdown);
    if (limitstatedown == 0) {
      digitalWrite(dirPin, LOW);
      while (digitalRead(limitup) == 1) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(speedo);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(speedo);
      }
      digitalWrite(dirPinclaw, LOW);
      for (int x = 0; x < 300; x++) {
        digitalWrite(stepPinclaw, HIGH);
        delayMicroseconds(speedoclaw);
        digitalWrite(stepPinclaw, LOW);
        delayMicroseconds(speedoclaw);
      }
    }

    if (distance2 < 15  && distance < 11 ) {
      place();
    } else if (distance < 9 && distance2 > 20 ) {
      pickupMechanism();
    }
  }
}

void calculateWheelSpeeds(float front, float vy, float omega, int speeds[]) {
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
  const float smoothingFactor = 0.5; // Adjust this value as needed for smoother movement
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

void stopMotors()
{
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
  digitalWrite(dirPin, HIGH);
  while (digitalRead(limitdown) == 1)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speedo);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speedo);
  }
  digitalWrite(dirPinclaw, HIGH);
  for (int x = 0; x < 300; x++)
  {
    digitalWrite(stepPinclaw, HIGH);
    delayMicroseconds(speedoclaw);
    digitalWrite(stepPinclaw, LOW);
    delayMicroseconds(speedoclaw);
  }
  digitalWrite(dirPin, LOW);
  while (digitalRead(limitup) == 1)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speedo);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speedo);
  }

  chisu = 1;

  servoAngle = 50;
  myservo.write(servoAngle);
  Serial.println(chisu);
}

void place() {
  stopMotors();
  digitalWrite(dirPinclaw, LOW);

  for (int x = 0; x < 300; x++)
  {
    digitalWrite(stepPinclaw, HIGH);
    delayMicroseconds(speedoclaw);
    digitalWrite(stepPinclaw, LOW);
    delayMicroseconds(speedoclaw);
  }

  for ( int i = 0 ; i < 20000 ; i++)
  {
    calculateWheelSpeeds(-40, 0, 0, speeds);
    setMotorSpeeds(speeds);
  }

  for ( int i = 0 ; i < 20000 ; i++)
  {
    calculateWheelSpeeds(0, 0, -30, speeds);
    setMotorSpeeds(speeds);
  }

  if (chisu != 0)
  { // Check if chisu has changed
    chisu = 0;
    Serial.println(chisu);
    //    myservo.write(22);
  }
}
