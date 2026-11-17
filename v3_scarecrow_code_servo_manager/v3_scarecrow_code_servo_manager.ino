#include <Servo.h>
#include "pitches.h"


// Pins
const int headServoPin = 8;
const int leftServoPin = 12;
const int rightServoPin = 11;

const int ultrasonic1TrigPin = 9;
const int ultrasonic1EchoPin = 10;
const int ultrasonic2TrigPin = 6;
const int ultrasonic2EchoPin = 7;

const int buzzerPin = 2;

const int PIRPin = 5;

const int ledPin = 3;


// Settings
const float ultrasonicDetectDistM = 0.5;
const unsigned long PIRWarmUpTime = 60000; // PIRs give false readings for ~60s after power on
const int timeBetweenSamples = 100;
const unsigned long servoSettleTime = 0; // extra wait after a servo finishes moving, before the servo manager starts the next one


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


// Classes
// Base class for anything that needs time to finish an action (servos moving, notes playing)
struct Cooldown {
  unsigned long nextFree = 0;

  void setTimeUntilFree(unsigned long timeUntilFree) {
    nextFree = millis() + timeUntilFree;
  }

  unsigned long getTimeUntilFree() {
    // Signed subtraction so this still works when millis() rolls over
    long remaining = (long)(nextFree - millis());
    return (remaining > 0) ? remaining : 0;
  }

  bool isFree() {
    return getTimeUntilFree() == 0;
  }
};

struct ServoMotor : Cooldown {
  Servo servo;
  int pin;
  int idleAngle, minAngle, maxAngle;
  int fullSweepTime;
  bool isPositionKnown;

  ServoMotor(int p, int iAngle = 90, int mnAngle = 0, int mxAngle = 180, int fullSweepT = 1000) {
    pin = p;
    idleAngle = iAngle;
    minAngle = min(mnAngle, mxAngle);
    maxAngle = max(mnAngle, mxAngle);
    fullSweepTime = fullSweepT;
    isPositionKnown = false;
  }

  void attachToPin() {
    if (!servo.attached()) {
      servo.attach(pin);
    }
  }

  void detachFromPin() {
    servo.detach();
  }

  // Turns immediately, ignoring the cooldown because ServoManager handles cooldowns now
  void turn(int angle) {
    int angleDiff = angle - servo.read();
    float sweepAngleRatio = abs(angleDiff)/(float)(maxAngle-minAngle);
    unsigned long sweepTime = sweepAngleRatio * fullSweepTime;

    // On the first move after power on, the servo could be anywhere, so assume a full sweep
    if (!isPositionKnown) {
      sweepTime = fullSweepTime;
      isPositionKnown = true;
    }

    setTimeUntilFree(sweepTime);

    // Write before attaching, so attaching doesn't snap to 90 first
    servo.write(angle);
    attachToPin();
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
};

// A single one-off servo movement, used by the servo manager's priority queue
struct ServoMove {
  ServoMotor* servoMotor;
  int angle;
};

// Only lets one servo move at a time, so their current spikes don't add up and reset the Arduino
// Priority queue: one-off moves (e.g. turning the head), done first, in the order they were added
// Cyclical queue: repeated sweeps (e.g. swinging the arms), only done while cycling and the priority queue is empty
struct ServoManager : Cooldown {
  static const int priorityQueueSize = 8;
  static const int cyclicalQueueSize = 4;

  unsigned long settleTime;

  ServoMove priorityQueue[priorityQueueSize];
  int priorityQueueStart;
  int priorityQueueLength;

  ServoMotor* cyclicalQueue[cyclicalQueueSize];
  int cyclicalQueueLength;
  int currentCycleIndex;
  bool isCycling;

  ServoManager(unsigned long sTime) {
    settleTime = sTime;
    priorityQueueStart = 0;
    priorityQueueLength = 0;
    cyclicalQueueLength = 0;
    currentCycleIndex = 0;
    isCycling = false;
  }

  // Returns false if the queue is full
  bool addToPriorityQueue(ServoMotor &servoMotor, int angle) {
    if (priorityQueueLength >= priorityQueueSize) {
      return false;
    }

    // The queue wraps around the end of the array
    int index = (priorityQueueStart + priorityQueueLength) % priorityQueueSize;
    priorityQueue[index].servoMotor = &servoMotor;
    priorityQueue[index].angle = angle;
    priorityQueueLength = priorityQueueLength + 1;

    return true;
  }

  // Returns false if the queue is full
  bool addToCyclicalQueue(ServoMotor &servoMotor) {
    if (cyclicalQueueLength >= cyclicalQueueSize) {
      return false;
    }

    cyclicalQueue[cyclicalQueueLength] = &servoMotor;
    cyclicalQueueLength = cyclicalQueueLength + 1;

    return true;
  }

  // Stopping and starting again resumes the cycle where it left off
  void startCycling() {
    isCycling = true;
  }

  void stopCycling() {
    isCycling = false;
  }

  bool isPriorityQueueEmpty() {
    return priorityQueueLength == 0;
  }

  // Starts the next move, but only once the previous one has finished
  void update() {
    if (!isFree()) {
      return;
    }

    if (!isPriorityQueueEmpty()) {
      ServoMove move = priorityQueue[priorityQueueStart];
      priorityQueueStart = (priorityQueueStart + 1) % priorityQueueSize;
      priorityQueueLength = priorityQueueLength - 1;

      move.servoMotor->turn(move.angle);
      waitForServo(*move.servoMotor);
    } else if (isCycling && cyclicalQueueLength > 0) {
      ServoMotor* servoMotor = cyclicalQueue[currentCycleIndex];
      currentCycleIndex = (currentCycleIndex + 1) % cyclicalQueueLength;

      (*servoMotor).turnSweep();
      waitForServo(*servoMotor);
    }
  }

  // The manager is free again once the servo has finished moving, plus a bit extra to settle, although the circuit seems to be fine with settleTime = 0
  void waitForServo(ServoMotor &servoMotor) {
    setTimeUntilFree(servoMotor.getTimeUntilFree() + settleTime);
  }

  // Blocks until every move in the priority queue has finished
  void waitUntilFinished() {
    while (!isPriorityQueueEmpty() || !isFree()) {
      update();
      delay(getTimeUntilFree());
    }
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

  // Returns 0 if there was no echo
  float getDistM() {
    // Clear trigger
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Send 10 µs pulse
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read echo time (µs), giving up after 30ms (~5m)
    long duration = pulseIn(echoPin, HIGH, 30000);

    // Calculate distance (m)
    float distance = duration * 0.034 / 2 / 100;

    recentValue = distance;

    return distance;
  }

  float getRecentValue() {
    return recentValue;
  }
};

struct Buzzer : Cooldown {
  int pin;
  const int* notes = nullptr;
  const int* durations = nullptr;
  int songLengthInElements;
  int currentNoteIndex;

  template <size_t N>
  Buzzer(int p, const int (&n)[N], const int (&d)[N]) {
    pin = p;
    notes = n;
    durations = d;
    songLengthInElements = N;
    currentNoteIndex = 0;
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

    if (notes[currentNoteIndex] != REST) {
      tone(pin, notes[currentNoteIndex], duration);
    }

    //to distinguish the notes, set a minimum time between them.
    //the note's duration + 30% seems to work well:
    int pauseBetweenNotes = duration * 1.30;
    setTimeUntilFree(pauseBetweenNotes);

    incrementCurrentNoteIndex();
  }

  // Plays the next note only once the previous one has finished
  void update() {
    if (isFree()) {
      playCurrentNote();
    }
  }

  bool isSongFinished() {
    return currentNoteIndex >= songLengthInElements;
  }

  // Finished AND the last note has finished playing
  bool isSongFinishedAndFree() {
    return isSongFinished() && isFree();
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

  LED(int p) {
    pin = p;
  }

  void setup() {
    pinMode(pin, OUTPUT);
  }

  void turn(int val) {
    val = constrain(val, LOW, HIGH);
    digitalWrite(pin, val);
  }
};

// Objects
ServoMotor headServo = ServoMotor(headServoPin, 0);
ServoMotor leftServo = ServoMotor(leftServoPin);
ServoMotor rightServo = ServoMotor(rightServoPin);

ServoManager servoManager = ServoManager(servoSettleTime);

UltrasonicSensor ultrasonic1 = UltrasonicSensor(ultrasonic1TrigPin, ultrasonic1EchoPin);
UltrasonicSensor ultrasonic2 = UltrasonicSensor(ultrasonic2TrigPin, ultrasonic2EchoPin);

Buzzer buzzer = Buzzer(buzzerPin, melody, durations);

PassiveInfraredSensor PIR = PassiveInfraredSensor(PIRPin);

LED led = LED(ledPin);

// State
enum State { CALM, ANGRY };
State state = CALM;

// Functions
void setupAllComponents() {
  ultrasonic1.setup();
  ultrasonic2.setup();

  buzzer.setup();

  PIR.setup();

  led.setup();

  // Arms take turns swinging while angry
  servoManager.addToCyclicalQueue(leftServo);
  servoManager.addToCyclicalQueue(rightServo);
}

void resetToIdle() {
  // The servo manager moves them one at a time, to avoid a big current spike
  servoManager.stopCycling();
  servoManager.addToPriorityQueue(headServo, headServo.idleAngle);
  servoManager.addToPriorityQueue(leftServo, leftServo.idleAngle);
  servoManager.addToPriorityQueue(rightServo, rightServo.idleAngle);
  servoManager.waitUntilFinished();

  // Detached servos don't buzz or draw current while idle
  headServo.detachFromPin();
  leftServo.detachFromPin();
  rightServo.detachFromPin();

  led.turn(LOW);

  buzzer.resetCurrentNoteIndex();
  buzzer.noTone();
}

bool isBirdNearby() {
  // Read values
  ultrasonic1.getDistM();
  ultrasonic2.getDistM();

  int recentPIRValue = PIR.getRecentValue();

  PIR.getMovement();

  // Process values
  float distM1 = ultrasonic1.getRecentValue();
  float distM2 = ultrasonic2.getRecentValue();
  bool didUltrasonic1Detect = distM1 > 0 && distM1 <= ultrasonicDetectDistM; // 0 means no echo
  bool didUltrasonic2Detect = distM2 > 0 && distM2 <= ultrasonicDetectDistM; // 0 means no echo


  bool arePIRsReady = millis() > PIRWarmUpTime;
  int currentPIRValue = PIR.getRecentValue();
  bool didPIRDetect = arePIRsReady && currentPIRValue == HIGH && recentPIRValue == LOW;
  // Serial.print(arePIRsReady);
  // Serial.print(": ");
  // Serial.println(didPIRDetect);

  // Final result
  return didUltrasonic1Detect || didUltrasonic2Detect || didPIRDetect;
}

void transitionToAngry() {
  state = ANGRY;

  led.turn(HIGH);

  buzzer.resetCurrentNoteIndex();
  buzzer.setTimeUntilFree(0);

  // Head turns first, then the arms start swinging
  servoManager.addToPriorityQueue(headServo, headServo.maxAngle);
  servoManager.startCycling();
}

void transitionToCalm() {
  state = CALM;
  resetToIdle();
}

void updateCalm() {
  if (isBirdNearby()) {
    transitionToAngry();
  } else {
    // 'Please wait' -Traffic Light (2026)
    delay(timeBetweenSamples);
  }
}

void updateAngry() {
  if (buzzer.isSongFinishedAndFree()) {
    // Return to happy/calm
    transitionToCalm();
    return;
  }

  // Each of these only acts if its cooldown has finished
  buzzer.update();
  servoManager.update();

  // Sleep until the next thing is free
  unsigned long timeUntilBuzzerIsFree = buzzer.getTimeUntilFree();
  unsigned long timeUntilServoManagerIsFree = servoManager.getTimeUntilFree();
  unsigned long timeUntilSomethingIsFree = min(timeUntilBuzzerIsFree, timeUntilServoManagerIsFree);
  delay(timeUntilSomethingIsFree);
}

void setup() {
  setupAllComponents();
  transitionToCalm();
}

void loop() {
  if (state == CALM) {
    updateCalm();
  } else {
    updateAngry();
  }
}
