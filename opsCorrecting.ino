#include <SoftwareSerial.h>

// Define the pins of the trigger pins and echo pins 
const int trigPin = 3;
const int echoPin = 12;

const int leftIn1 = 9;
const int leftIn2 = 10;
const int leftSpd = 6;

const int rightIn1 = 7;
const int rightIn2 = 8;
const int rightSpd = 5;

const int baseSpd = 200;

const float kp = 1;
const float kd = 0.1;

float PID;
float delta_distance;

float duration, distance, old_distance;

void setup() {
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(leftIn1, OUTPUT);
  pinMode(leftIn2, OUTPUT);

  pinMode(rightIn1, OUTPUT);
  pinMode(rightIn2, OUTPUT);

  digitalWrite(leftIn1, LOW);
  digitalWrite(leftIn2, HIGH);
  digitalWrite(rightIn1, HIGH);
  digitalWrite(rightIn2, LOW);

  old_distance = 0;
  Serial.begin(9600);
}

void loop() {
  
  //------- UltraSound -------
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); //Send pulse
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH); // Recieve pulse
  distance = (duration/2)* 0.0343 - 10;
  distance -= 2;

  //-------PID---------
  delta_distance = distance - old_distance;
  PID = kp * distance + kd * delta_distance;
  analogWrite(rightSpd, 150 + (int) PID);
  analogWrite(leftSpd, 175 - (int) PID);
  old_distance = distance;
  
}
