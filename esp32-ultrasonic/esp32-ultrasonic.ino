const int trigPin1 = 14;
const int echoPin1 = 12;
const int trigPin2 = 26;
const int echoPin2 = 27;
const int trigPin3 = 33;
const int echoPin3 = 25;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration1, duration2, duration3;
float distanceCm1, distanceCm2, distanceCm3;

void setup() {
  Serial.begin(115200); 
  pinMode(trigPin1, OUTPUT); 
  pinMode(echoPin1, INPUT); 
  pinMode(trigPin2, OUTPUT); 
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT); 
  pinMode(echoPin3, INPUT);
}

void loop() {
  // Clears the trigPin
  digitalWrite(trigPin1, LOW);
  digitalWrite(trigPin2, LOW);
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin1, HIGH);

  
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH); 
  
  
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH); 


  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  duration3 = pulseIn(echoPin3, HIGH); 
  // Reads the echoPin, returns the sound wave travel time in microseconds
  
  // Calculate the distance
  distanceCm1 = duration1 * SOUND_SPEED/2;
  distanceCm2 = duration2 * SOUND_SPEED/2;
  distanceCm3 = duration3 * SOUND_SPEED/2;
  
  Serial.print(distanceCm1);
  Serial.print(",");
  Serial.print(distanceCm2);
  Serial.print(",");
  Serial.print(distanceCm3);
  Serial.println();
  
  delay(1000);
}