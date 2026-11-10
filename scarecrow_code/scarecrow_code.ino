// #include <Servo.h>
// const int motionSensorPin = A5;

// int sensor_value = 0;

// void setup() {
//   Serial.begin(9600);
//   //pinMode(motionSensorPin, OUTPUT);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   //sensor_value = analogRead(motionSensorPin);
//   //Serial.println(sensor_value);
//   //delay(100);
// }

#include <Servo.h>
#include "pitches.h"

// Music Data
int melody[] = {
  NOTE_AS4, NOTE_AS4, NOTE_AS4,
  NOTE_F5, NOTE_C6,
  NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6,
  NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6,
  NOTE_AS5, NOTE_A5, NOTE_AS5, NOTE_G5, NOTE_C5, NOTE_C5, NOTE_C5,
  NOTE_F5, NOTE_C6,
  NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6,

  NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6,
  NOTE_AS5, NOTE_A5, NOTE_AS5, NOTE_G5, NOTE_C5, NOTE_C5,
  NOTE_D5, NOTE_D5, NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F5,
  NOTE_F5, NOTE_G5, NOTE_A5, NOTE_G5, NOTE_D5, NOTE_E5, NOTE_C5, NOTE_C5,
  NOTE_D5, NOTE_D5, NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F5,

  NOTE_C6, NOTE_G5, NOTE_G5, REST, NOTE_C5,
  NOTE_D5, NOTE_D5, NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F5,
  NOTE_F5, NOTE_G5, NOTE_A5, NOTE_G5, NOTE_D5, NOTE_E5, NOTE_C6, NOTE_C6,
  NOTE_F6, NOTE_DS6, NOTE_CS6, NOTE_C6, NOTE_AS5, NOTE_GS5, NOTE_G5, NOTE_F5,
  NOTE_C6
};

int durations[] = {
  8, 8, 8,
  2, 2,
  8, 8, 8, 2, 4,
  8, 8, 8, 2, 4,
  8, 8, 8, 2, 8, 8, 8,
  2, 2,
  8, 8, 8, 2, 4,

  8, 8, 8, 2, 4,
  8, 8, 8, 2, 8, 16,
  4, 8, 8, 8, 8, 8,
  8, 8, 8, 4, 8, 4, 8, 16,
  4, 8, 8, 8, 8, 8,

  8, 16, 2, 8, 8,
  4, 8, 8, 8, 8, 8,
  8, 8, 8, 4, 8, 4, 8, 16,
  4, 8, 4, 8, 4, 8, 4, 8,
  1
};

// Servos
struct Arm {
  Servo servo;
  int minAngle, maxAngle, idleAngle;
  bool isUp;
  unsigned long swingEnd;
};

Servo HServo;
const int HServoPin = 8;
const int HAngryAngle = 180;
const int HHappyAngle = 0;

Arm LServo = { Servo(), 0, 180, 90, true, 0 };
const int LServoPin = 9;

Arm RServo = { Servo(), 0, 180, 90, true, 0 };
const int RServoPin = 10;

// Ultrasonic
const int trigPin = 11;
const int echoPin = 12;

// Buzzer
const int buzzPin = 6;

void setup() {
  Serial.begin(9600);

  // Ultrasonic Setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Buzzer Setup
  pinMode(buzzPin, OUTPUT);

  become_happy();
}

void loop() {
  bool canSeeBird = false;
  // Sensor Input
  float ultrasonic_distance = get_ultrasonic_distance();
  log_ultrasonic_distance(ultrasonic_distance);

  //Process Sensor Data
  if (ultrasonic_distance < 0.3) {
    canSeeBird = true;
  }

  // Control Output
  if (canSeeBird) {
    become_angry();
  }

  // Logging
  //log_ultrasonic_distance(ultrasonic_distance);
  
  delay(100);
}

// INPUT FUNCTIONS ----------------------------------------------------------

float get_ultrasonic_distance() {
  // Clear trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send 10 µs pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo time (µs)
  long duration = pulseIn(echoPin, HIGH);

  // Calculate distance (m)
  float distance = duration * 0.034 / 2 / 100;

  return distance;
}

bool has_arm_finished_swinging(Arm &arm) {
  return (long)(millis() - arm.swingEnd) >= 0;
}

// CONTROL FUNCTIONS ----------------------------------------------------------

void become_angry() {
  RServo.servo.attach(RServoPin);
  LServo.servo.attach(LServoPin);
  HServo.attach(HServoPin);
  HServo.write(HAngryAngle);
  delay(200);
  HServo.detach();
  turn_on_led();

  swing_arm(LServo);
  swing_arm(RServo);

  int size = sizeof(durations) / sizeof(int);

  for (int note = 0; note < size; note++) {
    play_note(note);

    if (has_arm_finished_swinging(LServo)) {
      swing_arm(LServo);
    }

    if (has_arm_finished_swinging(RServo)) {
      swing_arm(RServo);
    }
  }

  delay(500);
  LServo.servo.detach();
  RServo.servo.detach();
  become_happy();
}

void become_happy() {
  HServo.attach(HServoPin);
  LServo.servo.attach(LServoPin);
  RServo.servo.attach(RServoPin);

  LServo.servo.write(LServo.idleAngle);
  RServo.servo.write(RServo.idleAngle);
  disable_led();
  HServo.write(HHappyAngle);
  delay(200);
  LServo.servo.detach();
  RServo.servo.detach();
  HServo.detach();
}

void swing_arm(Arm &arm) {
  int desiredAngle = arm.isUp ? arm.minAngle : arm.maxAngle;
  arm.swingEnd = millis() + abs(desiredAngle - arm.servo.read()) * 2 + 300;
  arm.servo.write(desiredAngle);
  arm.isUp = !arm.isUp;
}

void disable_led() {
  return;
}

void turn_on_led() {
  return;
}

void play_note(int note_index) {
  //to calculate the note duration, take one second divided by the note type.
  //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
  int duration = 1000 / durations[note_index];
  tone(buzzPin, melody[note_index], duration);

  //to distinguish the notes, set a minimum time between them.
  //the note's duration + 30% seems to work well:
  int pauseBetweenNotes = duration * 1.30;
  delay(pauseBetweenNotes);

  //stop the tone playing:
  noTone(buzzPin);
}

// LOGGING FUNCTIONS ----------------------------------------------------------

void log_ultrasonic_distance(float distance_m) {
  Serial.print("Distance: ");
  Serial.print(distance_m);
  Serial.println(" m");
}
