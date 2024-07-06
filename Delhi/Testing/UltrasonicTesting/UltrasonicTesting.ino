//Ultra sonic down
const int trigPin1 = 30;
const int echoPin1 = 6;
long duration1;
int distance1;

//Ultra sonic up
const int trigPin2 = 12;
const int echoPin2 = 13;
long duration2;
int distance2;

void setup()
{
  Serial.begin(9600);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
}

void loop()
{
  //Write a pulse to the HC-SR04 Trigger Pin
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  // Measure the response from the HC-SR04 Echo Pin
  duration1 = pulseIn(echoPin1, HIGH);

  //Determine distance from duration
  //Use 343 metres per second as speed of sound
  distance1 = duration1 * (0.034 / 2);

  //Write a pulse to the HC-SR04 Trigger Pin
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);

  // Measure the response from the HC-SR04 Echo Pin
  duration2 = pulseIn(echoPin2, HIGH);

  //Determine distance from duration
  //Use 343 metres per second as speed of sound
  distance2 = duration2 * (0.034 / 2);

  Serial.print("Distance1: ");
  Serial.print(distance1);
  Serial.print(" cm || ");
  Serial.print("Distance2: ");
  Serial.print(distance2);
  Serial.println(" cm.");
  delay(10);
}
