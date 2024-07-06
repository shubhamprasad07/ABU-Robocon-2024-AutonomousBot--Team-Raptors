const int trigPin = 12;
const int echoPin = 13;
long duration;
int distance;

void setup()
{
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop()
{
  //Write a pulse to the HC-SR04 Trigger Pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the response from the HC-SR04 Echo Pin
  duration = pulseIn(echoPin, HIGH);

  //Determine distance from duration
  //Use 343 metres per second as speed of sound
  distance = duration * (0.034 / 2);

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm.");
  delay(100);
}
