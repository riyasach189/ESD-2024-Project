#include <Arduino.h>

const int trigPin1 = 14; // Left sensor
const int echoPin1 = 12;
const int trigPin2 = 26; // Front sensor
const int echoPin2 = 27;
const int trigPin3 = 33; // Right sensor
const int echoPin3 = 25;

#define SOUND_SPEED 0.034
#define OBSTACLE_THRESHOLD_CM 30 // Change this based on your need

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
  distanceCm1 = measureDistance(trigPin1, echoPin1);
  distanceCm2 = measureDistance(trigPin2, echoPin2);
  distanceCm3 = measureDistance(trigPin3, echoPin3);

  if (distanceCm1 > OBSTACLE_THRESHOLD_CM && distanceCm2 > OBSTACLE_THRESHOLD_CM && distanceCm3 > OBSTACLE_THRESHOLD_CM) {
    forward();
  } else if (distanceCm1 <= OBSTACLE_THRESHOLD_CM && distanceCm2 > OBSTACLE_THRESHOLD_CM) {
    // Obstacle on the left, move straight
    forward();
  } else if ((distanceCm1 > OBSTACLE_THRESHOLD_CM && distanceCm2 <= OBSTACLE_THRESHOLD_CM) || (distanceCm1 > OBSTACLE_THRESHOLD_CM && distanceCm3 <= OBSTACLE_THRESHOLD_CM)) {
    // Obstacle straight ahead or on the right, move left
    left();
  } else if (distanceCm1 <= OBSTACLE_THRESHOLD_CM && distanceCm2 <= OBSTACLE_THRESHOLD_CM) {
    // Obstacle on left and straight ahead, move right
    right();
  } else {
    // Handle other situations or stop
    stop(); // You need to define this function
  }

  delay(1000); // Adjust based on how frequently you want to check for obstacles
}

float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * SOUND_SPEED / 2;
}

void moveLeft() {
  // Implement movement logic
  Serial.println("Moving Left");
}

void moveStraight() {
  // Implement movement logic
  Serial.println("Moving Straight");
}

void moveRight() {
  // Implement movement logic
  Serial.println("Moving Right");
}

void stopMoving() {
  // Implement stop logic
  Serial.println("Stopping");
}
