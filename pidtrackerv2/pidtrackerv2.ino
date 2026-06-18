#include <ESP32Servo.h>

#define SERVO_PIN   13
#define TRIG_PIN     5
#define ECHO_PIN    18

// --- Sweep settings ---
const float DETECTION_RANGE_CM = 30.0;
const int   SWEEP_DELAY_MS     = 5;      // faster sweep
const int   LOST_WAIT_MS       = 500;    // faster resume

// --- PID tuning constants ---
const float Kp = 0.8;
const float Ki = 0.0;
const float Kd = 0.3;

// --- Tracking settings ---
const int   TRACK_STEP_DEG    = 3;
const int   TRACK_DELAY_MS    = 40;
const float DEADBAND_CM       = 2.0;
const float MAX_CORRECTION    = 10.0;

Servo myServo;

int  currentAngle   =  0;
int  sweepDirection =  1;
int  lastDirection  =  1;

float pidIntegral   = 0.0;
float pidLastError  = 0.0;
unsigned long lastPidTime = 0;

enum Mode { SWEEP, TRACK };
Mode mode = SWEEP;

// -----------------------------------------------------------------------
// HELPERS
// -----------------------------------------------------------------------

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
  currentAngle = angle;
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

void resetPID() {
  pidIntegral  = 0.0;
  pidLastError = 0.0;
  lastPidTime  = millis();
}

// -----------------------------------------------------------------------
// PID TRACKER
// -----------------------------------------------------------------------

bool trackPID() {
  int leftAngle   = constrain(currentAngle - TRACK_STEP_DEG, 0, 180);
  int rightAngle  = constrain(currentAngle + TRACK_STEP_DEG, 0, 180);

  // Sample left, center, and right
  float leftDist   = sampleDistanceAt(leftAngle);
  setAngle(currentAngle);
  delay(20);
  float centerDist = sampleDistanceAt(currentAngle);  // <-- new center check
  delay(20);
  float rightDist  = sampleDistanceAt(rightAngle);
  setAngle(currentAngle);
  delay(20);

  bool leftDetected   = (leftDist   <= DETECTION_RANGE_CM);
  bool centerDetected = (centerDist <= DETECTION_RANGE_CM);
  bool rightDetected  = (rightDist  <= DETECTION_RANGE_CM);

  // Object lost on ALL three positions
  if (!leftDetected && !centerDetected && !rightDetected) return false;

  // --- Calculate error ---
  float error;

  if (centerDetected && !leftDetected && !rightDetected) {
    // Perfectly centered — no correction needed
    error = 0.0;
  } else if (leftDetected && !rightDetected) {
    error = -5.0;   // strong pull left
  } else if (rightDetected && !leftDetected) {
    error = 5.0;    // strong pull right
  } else {
    error = rightDist - leftDist;  // proportional difference
  }

  // Deadband
  if (abs(error) < DEADBAND_CM) {
    error = 0.0;
  }

  // --- Time delta ---
  unsigned long now = millis();
  float dt = (now - lastPidTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  lastPidTime = now;

  // --- P term ---
  float P = Kp * error;

  // --- I term ---
  pidIntegral += error * dt;
  pidIntegral = constrain(pidIntegral, -20.0, 20.0);
  float I = Ki * pidIntegral;

  // --- D term ---
  float derivative = (error - pidLastError) / dt;
  float D = Kd * derivative;
  pidLastError = error;

  // --- Total correction ---
  float correction = P + I + D;
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  int newAngle = constrain(currentAngle + (int)correction, 0, 180);
  setAngle(newAngle);

  if (correction > 0) lastDirection =  1;
  if (correction < 0) lastDirection = -1;

  Serial.print("[TRACK]  Angle: ");
  Serial.print(currentAngle);
  Serial.print("°  |  L: ");
  Serial.print(leftDist, 1);
  Serial.print("  C: ");
  Serial.print(centerDist, 1);
  Serial.print("  R: ");
  Serial.print(rightDist, 1);
  Serial.print("  |  Error: ");
  Serial.print(error, 1);
  Serial.print("  |  Correction: ");
  Serial.println(correction, 2);

  return true;
}

// -----------------------------------------------------------------------
// SETUP & MAIN LOOP
// -----------------------------------------------------------------------

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
      delay(300);
    } else if (currentAngle <= 0) {
      currentAngle   = 0;
      sweepDirection = 1;
      delay(300);
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
      resetPID();
      mode = TRACK;
    }

  } else if (mode == TRACK) {
    bool stillTracking = trackPID();

    if (!stillTracking) {
      Serial.print("\n*** Object lost — waiting ");
      Serial.print(LOST_WAIT_MS / 1000.0, 1);
      Serial.println("s before resuming sweep ***\n");
      delay(LOST_WAIT_MS);
      mode = SWEEP;
    }
  }
}