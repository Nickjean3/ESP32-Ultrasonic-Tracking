#include <ESP32Servo.h>

#define SERVO_PIN   13
#define TRIG_PIN     5
#define ECHO_PIN    18

const float DETECTION_RANGE_CM = 30.0;
const int   SWEEP_DELAY_MS     = 10;
const int   TRACK_STEP_DEG     = 3;
const int   TRACK_DELAY_MS     = 50;
const int   LOST_WAIT_MS       = 2000;
const float TRACK_DEADBAND_CM  = 3.0;

Servo myServo;

int  currentAngle   =  0;
int  sweepDirection =  1;
int  lastDirection  =  1;

enum Mode { SWEEP, TRACK };
Mode mode = SWEEP;

struct TrackResult {
  int  newAngle;
  int  newDirection;
  bool lost;
};

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999.0;
  return duration * 0.01715;
}

void setAngle(int angle) {
  angle = constrain(angle, 0, 180);
  myServo.write(angle);
}

float sampleDistanceAt(int angle, int samples = 3) {
  setAngle(angle);
  delay(TRACK_DELAY_MS);
  float total = 0;
  for (int i = 0; i < samples; i++) {
    total += getDistance();
    delay(5);
  }
  return total / samples;
}

TrackResult track(int currentAngle, int lastDir) {
  int leftAngle  = constrain(currentAngle - TRACK_STEP_DEG, 0, 180);
  int rightAngle = constrain(currentAngle + TRACK_STEP_DEG, 0, 180);

  float leftDist = sampleDistanceAt(leftAngle);
  setAngle(currentAngle);
  delay(20);
  float rightDist = sampleDistanceAt(rightAngle);
  setAngle(currentAngle);
  delay(20);

  bool leftDetected  = (leftDist  <= DETECTION_RANGE_CM);
  bool rightDetected = (rightDist <= DETECTION_RANGE_CM);

  if (!leftDetected && !rightDetected) return { currentAngle, lastDir, true };
  if (leftDetected  && !rightDetected) return { leftAngle,  -1, false };
  if (rightDetected && !leftDetected)  return { rightAngle,  1, false };

  float diff = leftDist - rightDist;
  if (abs(diff) <= TRACK_DEADBAND_CM) {
    int newAngle = constrain(currentAngle + (lastDir * TRACK_STEP_DEG), 0, 180);
    return { newAngle, lastDir, false };
  }

  if (leftDist < rightDist) return { leftAngle,  -1, false };
  else                      return { rightAngle,  1, false };
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  ESP32PWM::allocateTimer(0);
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2500);

  setAngle(currentAngle);
  delay(500);

  Serial.println("Program is starting...");
  Serial.print("Detection range: ");
  Serial.print(DETECTION_RANGE_CM);
  Serial.println(" cm");
}

void loop() {
  if (mode == SWEEP) {
    currentAngle += sweepDirection;

    if (currentAngle >= 180) {
      currentAngle   = 180;
      sweepDirection = -1;
      delay(500);
    } else if (currentAngle <= 0) {
      currentAngle   = 0;
      sweepDirection = 1;
      delay(500);
    }

    setAngle(currentAngle);
    float dist = getDistance();

    Serial.print("[SWEEP]  Angle: ");
    Serial.print(currentAngle);
    Serial.print("°  |  Distance: ");
    Serial.print(dist, 1);
    Serial.println(" cm");

    delay(SWEEP_DELAY_MS);

    if (dist <= DETECTION_RANGE_CM) {
      Serial.print("\n*** Object detected at ");
      Serial.print(dist, 1);
      Serial.println(" cm — switching to TRACK mode ***\n");
      lastDirection = sweepDirection;
      mode = TRACK;
    }

  } else if (mode == TRACK) {
    TrackResult result = track(currentAngle, lastDirection);
    lastDirection = result.newDirection;

    if (result.lost) {
      Serial.print("\n*** Object lost — waiting ");
      Serial.print(LOST_WAIT_MS / 1000.0, 1);
      Serial.println("s before resuming sweep ***\n");
      delay(LOST_WAIT_MS);
      mode = SWEEP;
    } else {
      currentAngle = result.newAngle;
      float dist = getDistance();
      const char* dirLabel = (lastDirection == -1) ? "CCW" : "CW";

      Serial.print("[TRACK]  Angle: ");
      Serial.print(currentAngle);
      Serial.print("°  |  Distance: ");
      Serial.print(dist, 1);
      Serial.print(" cm  |  Moving: ");
      Serial.println(dirLabel);
    }
  }
}