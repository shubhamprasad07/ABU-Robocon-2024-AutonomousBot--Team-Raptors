#define MOTOR1_PWM 2  // Front left wheel
#define MOTOR1_DIR 22

#define MOTOR2_PWM 3  // Front right wheel
#define MOTOR2_DIR 23

#define MOTOR3_PWM 4  // Rear left wheel
#define MOTOR3_DIR 24

#define MOTOR4_PWM 5  // Rear right wheel
#define MOTOR4_DIR 25

float vy;
float vx;
int omega = 0;

void calculateWheelSpeeds(float vx, float vy, float omega, int speeds[]);
void setMotorSpeeds(int speeds[]);
void stopMotors();

void setup() {

  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_DIR, OUTPUT);
  pinMode(MOTOR4_PWM, OUTPUT);
  pinMode(MOTOR4_DIR, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Read data from serial buffer if available
  while (Serial.available() == 0) {
  }
  //vx = 50;
  // Read the incoming bytes until a newline character is received
  String data = Serial.readStringUntil('\r');

  // Extract x, y, and area coordinates from the received string
  int firstCommaIndex = data.indexOf(',');
  int secondCommaIndex = data.indexOf(',', firstCommaIndex + 1);
  String x_str = data.substring(0, firstCommaIndex);
  String y_str = data.substring(firstCommaIndex + 1, secondCommaIndex);
  String area_str = data.substring(secondCommaIndex + 1);

  // Convert strings to integers
  int x_coordinate = x_str.toInt();
  int y_coordinate = y_str.toInt();
  int area_value = area_str.toInt();
  omega = map(x_coordinate, -160, 160, -50, 50);
  vy = y_coordinate;
  //  vx = map(area_value, 43, 0, -50, 50);
  if (vy == 0) {
    vx = 0;
    stopMotors();
    
  }
  else {
    vx = 50;
  }
  if (omega > 15 || omega < -15 || vx > 0) {
    int speeds[4];
    calculateWheelSpeeds(vx, vy, omega, speeds);
    setMotorSpeeds(speeds);
  }
  else {
    stopMotors();
  }
}

void calculateWheelSpeeds(float vx, float vy, float omega, int speeds[]) {
  speeds[0] = (vx + omega);   // Front left wheel
  speeds[1] = (vx - omega)-10;   // Front right wheel
  speeds[2] = (vx - omega)-10;  // Rear left wheel
  speeds[3] = (vx + omega);  // Rear right wheel
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
