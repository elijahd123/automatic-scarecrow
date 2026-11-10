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
  NOTE_C6,

  REST
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
  1,

  1
};

// Classes
struct ServoMotor {
  Servo servo;
  int pin;
  int idleAngle, minAngle, maxAngle;
  int fullSweepTime;
  unsigned long nextFree;

  ServoMotor(int p, int iAngle = 90, int mnAngle = 0, int mxAngle = 180, int fullSweepT = 1000) {
    pin = p;
    idleAngle = iAngle;
    minAngle = min(mnAngle, mxAngle);
    maxAngle = max(mnAngle, mxAngle);
    fullSweepTime = fullSweepT;
    nextFree = millis();
  }

  void attachToPin() {
    servo.attach(pin);
  }

  void detachFromPin() {
    servo.detach();
  }

  void turn(int angle) {
    int angleDiff = angle - servo.read();
    float sweepAngleRatio = abs(angleDiff)/(float)(maxAngle-minAngle);
    unsigned long sweepTime = sweepAngleRatio * fullSweepTime;

    setTimeUntilFree(sweepTime);

    attachToPin();
    servo.write(angle);
  }

  void turnToIdle() {
    turn(idleAngle);
  }

  void turnToMin() {
    turn(minAngle);
  }

  void turnToMax() {
    turn(maxAngle);
  }

  void turnSweep() {
    if (servo.read() == maxAngle) {
      turnToMin();
    } else {
      turnToMax();
    }
  }

  bool isFree() {
    return millis() >= nextFree;
  }

  void setTimeUntilFree(unsigned long timeUntilFree) {
    nextFree = millis() + timeUntilFree;
  }

  unsigned long getTimeUntilFree() {
    unsigned long now = millis();
    return (now >= nextFree) ? 0 : nextFree - now;
  }
};

struct UltrasonicSensor {
  int trigPin;
  int echoPin;
  float recentValue;

  UltrasonicSensor(int tPin, int ePin) {
    trigPin = tPin;
    echoPin = ePin;
    recentValue = -1.0;
  }

  void setup() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }

  float getDistM() {
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

    recentValue = distance;

    return distance;
  }

  float getrecentValue() {
    return recentValue;
  }
};

struct Buzzer {
  int pin;
  const int* notes = nullptr;
  const int* durations = nullptr;
  int songLengthInElements;
  int currentNoteIndex;
  unsigned long nextFree;

  template <size_t N>
  Buzzer(int p, const int (&n)[N], const int (&d)[N]) {
    pin = p;
    notes = n;
    durations = d;
    songLengthInElements = N;
    currentNoteIndex = 0;
    nextFree = millis();
  }

  void setup() {
    pinMode(pin, OUTPUT);
  }

  void playCurrentNote() {
    if (isSongFinished()) {
      return;
    }

    //to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int duration = 1000 / durations[currentNoteIndex];
    tone(pin, notes[currentNoteIndex], duration);

    //to distinguish the notes, set a minimum time between them.
    //the note's duration + 30% seems to work well:
    int pauseBetweenNotes = duration * 1.30;
    setTimeUntilFree(pauseBetweenNotes);

    incrementCurrentNoteIndex();
  }

  bool isSongFinished() {
    return currentNoteIndex >= songLengthInElements;
  }

  void noTone() {
    //stop the tone playing:
    ::noTone(pin);
  }

  void resetCurrentNoteIndex() {
    currentNoteIndex = 0;
  }

  void incrementCurrentNoteIndex() {
    currentNoteIndex = currentNoteIndex + 1;
  }

  bool isFree() {
    return millis() >= nextFree;
  }

  void setTimeUntilFree(unsigned long timeUntilFree) {
    nextFree = millis() + timeUntilFree;
  }

  unsigned long getTimeUntilFree() {
    unsigned long now = millis();
    return (now >= nextFree) ? 0 : nextFree - now;
  }
};

// remember to set sensor to H i believe
struct PassiveInfraredSensor {
  int pin;
  int recentValue;

  PassiveInfraredSensor(int p) {
    pin = p;
    recentValue = LOW;
  }

  void setup() {
    pinMode(pin, INPUT);
  }

  int getMovement() {
    int value = digitalRead(pin);
    
    recentValue = value;

    return value;
  }

  int getRecentValue() {
    return recentValue;
  }
};

struct LED {
  int pin;
  unsigned long nextFree;

  LED(int p) {
    pin = p;
    nextFree = millis();
  }

  void setup() {
    pinMode(pin, OUTPUT);
  }

  void turn(int val) {
    if (!isFree()) {
      delay(getTimeUntilFree());
    }

    val = constrain(val, LOW, HIGH);

    digitalWrite(pin, val);

    setTimeUntilFree(500);
  }

  bool isFree() {
    return millis() >= nextFree;
  }

  void setTimeUntilFree(unsigned long timeUntilFree) {
    nextFree = millis() + timeUntilFree;
  }

  unsigned long getTimeUntilFree() {
    unsigned long now = millis();
    return (now >= nextFree) ? 0 : nextFree - now;
  }
};

// Objects
ServoMotor headServo = ServoMotor(8, 0);
ServoMotor leftServo = ServoMotor(9);
ServoMotor rightServo = ServoMotor(10);

UltrasonicSensor ultrasonic = UltrasonicSensor(11, 12);

Buzzer buzzer = Buzzer(6, melody, durations);

PassiveInfraredSensor PIR1 = PassiveInfraredSensor(4);
PassiveInfraredSensor PIR2 = PassiveInfraredSensor(A0);
PassiveInfraredSensor PIR3 = PassiveInfraredSensor(A1);

LED led1 = LED(3);
LED led2 = LED(4);

// Functions
void fullSetup() {
  ultrasonic.setup();

  buzzer.setup();

  PIR1.setup();
  PIR2.setup();
  PIR3.setup();

  led1.setup();
  led2.setup();
  

  headServo.turnToIdle();
  delay(500);
  leftServo.turnToIdle();
  delay(500);
  rightServo.turnToIdle();
  delay(500);

  led1.turn(LOW);
  led2.turn(LOW);

  headServo.detachFromPin();
  leftServo.detachFromPin();
  rightServo.detachFromPin();

  buzzer.resetCurrentNoteIndex();
  buzzer.noTone();
}

// Variables
bool isAngry = false;
int timeBetweenSamples = 100;

void setup() {
  Serial.begin(9600);

  fullSetup();
}

void loop() {
  if (!isAngry) {
    // Read values
    ultrasonic.getDistM();
    PIR1.getMovement();
    PIR2.getMovement();
    PIR3.getMovement();

    // Process values
    bool didUltrasonicDetect = ultrasonic.getrecentValue() <= 0.5;
    bool didPIR1Detect = PIR1.getRecentValue();
    bool didPIR2Detect = PIR2.getRecentValue();
    bool didPIR3Detect = PIR3.getRecentValue();

    // Final result
    isAngry = didUltrasonicDetect || didPIR1Detect || didPIR2Detect || didPIR3Detect;

    // 'Please wait' -Traffic Light (2026)
    delay(timeBetweenSamples);
    if (isAngry) {
    // Setup for angry
    headServo.turnToMax();
    delay(500);
    led1.turn(HIGH);
    led2.turn(HIGH);
    leftServo.setTimeUntilFree(0);
    rightServo.setTimeUntilFree(500);
    }
  } else { // isAngry = true; (or at least it's not false, which means either it's true or I've really screwed this up)
    if (buzzer.isSongFinished()) {
      // Return to happy/calm
      isAngry = false;
      fullSetup();
    } else {
      // Update angry stuff
      unsigned long timeUntilBuzzerIsFree = buzzer.getTimeUntilFree();
      unsigned long timeUntilLeftServoIsFree = leftServo.getTimeUntilFree();
      unsigned long timeUntilRightServoIsFree = rightServo.getTimeUntilFree();
      unsigned long timeUntilSomethingIsFree = 1 + min(timeUntilBuzzerIsFree, min(timeUntilLeftServoIsFree, timeUntilRightServoIsFree));
      delay(timeUntilSomethingIsFree);

      if (buzzer.getTimeUntilFree() <= 0) {
        buzzer.playCurrentNote();
      }

      if (leftServo.getTimeUntilFree() <= 0) {
        leftServo.turnSweep();
      }

      if (rightServo.getTimeUntilFree() <= 0) {
        rightServo.turnSweep();
      }
    }
  }
}