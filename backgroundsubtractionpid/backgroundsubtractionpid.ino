#include <ESP32Servo.h>

#define SERVO_PIN   13
#define TRIG_PIN     5
#define ECHO_PIN    18
#define LED_GREEN   26
#define LED_RED     27

// --- Sweep resolution ---
const int   SWEEP_STEP_DEG      = 3;
const int   SWEEP_DELAY_MS      = 35;

// --- Background subtraction ---
const float INTRUSION_THRESH_CM = 15.0;
const int   BG_SAMPLES          = 3;

// --- Tracking ---
const int   RESCAN_INTERVAL_MS  = 300;
const int   LOST_WAIT_MS        = 500;

// --- PID ---
float Kp = 1.2;
float Ki = 0.0;
float Kd = 0.4;
const float MAX_CORRECTION      = 8.0;
const float DEADBAND_DEG        = 1.0;

// --- Startup warning ---
const int   STARTUP_WARN_SEC    = 5;

Servo myServo;

float bgMap[61];

int   currentAngle    = 0;
int   sweepDirection  = 1;
int   targetAngle     = 0;

float pidIntegral     = 0.0;
float pidLastError    = 0.0;
unsigned long lastPidTime     = 0;
unsigned long lastRescanTime  = 0;
unsigned long lastBlinkTime   = 0;
bool  blinkState = false;

enum Mode { STARTUP, CALIBRATE, SWEEP, TRACK };
Mode mode = STARTUP;

// -----------------------------------------------------------------------
// LED HELPERS
// -----------------------------------------------------------------------

void setLeds(bool green, bool red) {
  digitalWrite(LED_GREEN, green ? HIGH : LOW);
  digitalWrite(LED_RED,   red   ? HIGH : LOW);
}

// Call repeatedly in a loop to blink without using delay()
void blinkLed(int pin, int intervalMs) {
  unsigned long now = millis();
  if (now - lastBlinkTime >= intervalMs) {
    lastBlinkTime = now;
    blinkState = !blinkState;
    digitalWrite(pin, blinkState ? HIGH : LOW);
  }
}

// -----------------------------------------------------------------------
// HELPERS
// -----------------------------------------------------------------------

int angleToIndex(int angle) {
  return angle / SWEEP_STEP_DEG;
}

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

float getAveragedDistance(int samples = 3) {
  float total = 0;
  for (int i = 0; i < samples; i++) {
    total += getDistance();
    delay(30);
  }
  return total / samples;
}

void setAngle(int angle) {
  angle = constrain(angle, 0, 180);
  myServo.write(angle);
  currentAngle = angle;
}

void resetPID() {
  pidIntegral  = 0.0;
  pidLastError = 0.0;
  lastPidTime  = millis();
}

// -----------------------------------------------------------------------
// BACKGROUND CALIBRATION
// -----------------------------------------------------------------------
void calibrateBackground() {
  Serial.println("\n=== BACKGROUND CALIBRATION ===");
  Serial.println("Scanning environment...");

  // Green on solid during calibration scan
  setLeds(true, false);

  for (int angle = 0; angle <= 180; angle += SWEEP_STEP_DEG) {
    setAngle(angle);
    delay(SWEEP_DELAY_MS);

    float dist = getAveragedDistance(BG_SAMPLES);
    bgMap[angleToIndex(angle)] = dist;

    Serial.print("  Mapped ");
    Serial.print(angle);
    Serial.print("° = ");
    Serial.print(dist, 1);
    Serial.println(" cm");
  }

  Serial.println("=== CALIBRATION COMPLETE — starting sweep ===\n");
}

// -----------------------------------------------------------------------
// INTRUSION CHECK
// -----------------------------------------------------------------------
bool isIntrusion(int angle, float currentDist) {
  float bgDist = bgMap[angleToIndex(angle)];
  if (bgDist >= 999.0) return currentDist < 200.0;
  return (bgDist - currentDist) >= INTRUSION_THRESH_CM;
}

// -----------------------------------------------------------------------
// CENTROID FINDER
// -----------------------------------------------------------------------
int findCentroid(int startAngle) {
  int leftEdge  = -1;
  int rightEdge = -1;

  for (int a = startAngle; a >= 0; a -= SWEEP_STEP_DEG) {
    setAngle(a);
    delay(SWEEP_DELAY_MS);
    if (isIntrusion(a, getDistance())) leftEdge = a;
    else break;
  }

  for (int a = startAngle; a <= 180; a += SWEEP_STEP_DEG) {
    setAngle(a);
    delay(SWEEP_DELAY_MS);
    if (isIntrusion(a, getDistance())) rightEdge = a;
    else break;
  }

  if (leftEdge == -1 && rightEdge == -1) return -1;
  if (leftEdge  == -1) leftEdge  = startAngle;
  if (rightEdge == -1) rightEdge = startAngle;

  int centroid = (leftEdge + rightEdge) / 2;

  Serial.print("[SCAN]  Left: ");
  Serial.print(leftEdge);
  Serial.print("°  Right: ");
  Serial.print(rightEdge);
  Serial.print("°  Centroid: ");
  Serial.print(centroid);
  Serial.println("°");

  return centroid;
}

// -----------------------------------------------------------------------
// PID
// -----------------------------------------------------------------------
void updatePID() {
  float error = targetAngle - currentAngle;
  if (abs(error) < DEADBAND_DEG) error = 0.0;

  unsigned long now = millis();
  float dt = (now - lastPidTime) / 1000.0;
  if (dt <= 0.0) dt = 0.001;
  lastPidTime = now;

  float P = Kp * error;

  pidIntegral += error * dt;
  pidIntegral  = constrain(pidIntegral, -20.0, 20.0);
  float I = Ki * pidIntegral;

  float derivative = (error - pidLastError) / dt;
  float D = Kd * derivative;
  pidLastError = error;

  float correction = constrain(P + I + D, -MAX_CORRECTION, MAX_CORRECTION);
  setAngle(constrain(currentAngle + (int)correction, 0, 180));

  Serial.print("[TRACK]  Angle: ");
  Serial.print(currentAngle);
  Serial.print("°  Target: ");
  Serial.print(targetAngle);
  Serial.print("°  Error: ");
  Serial.print(error, 1);
  Serial.print("°  Correction: ");
  Serial.println(correction, 2);
}

// -----------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN,  OUTPUT);
  pinMode(ECHO_PIN,  INPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED,   OUTPUT);

  ESP32PWM::allocateTimer(0);
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2500);

  setAngle(0);
  setLeds(false, false);
  delay(500);

  Serial.println("Program is starting...");
  Serial.print("Startup warning: ");
  Serial.print(STARTUP_WARN_SEC);
  Serial.println(" seconds — clear the area!");

  mode = STARTUP;
}

// -----------------------------------------------------------------------
// MAIN LOOP
// -----------------------------------------------------------------------
void loop() {

  // --- STARTUP WARNING PHASE ---
  if (mode == STARTUP) {
    static unsigned long startupBegin = millis();

    unsigned long elapsed = millis() - startupBegin;
    int remaining = STARTUP_WARN_SEC - (elapsed / 1000);

    // Blink red rapidly during warning
    setLeds(false, false);
    blinkLed(LED_RED, 300);

    // Print countdown once per second
    static int lastPrinted = -1;
    if (remaining != lastPrinted && remaining >= 0) {
      Serial.print("Starting in ");
      Serial.print(remaining);
      Serial.println("...");
      lastPrinted = remaining;
    }

    if (elapsed >= (unsigned long)(STARTUP_WARN_SEC * 1000)) {
      setLeds(false, false);
      Serial.println();
      mode = CALIBRATE;
    }
    return;
  }

  // --- CALIBRATION PHASE ---
  if (mode == CALIBRATE) {
    calibrateBackground();
    mode = SWEEP;
    return;
  }

  // --- SWEEP MODE ---
  if (mode == SWEEP) {
    // Blink green slowly during sweep
    blinkLed(LED_GREEN, 500);
    digitalWrite(LED_RED, LOW);

    currentAngle += sweepDirection * SWEEP_STEP_DEG;

    if (currentAngle >= 180) {
      currentAngle   = 180;
      sweepDirection = -1;
      delay(200);
    } else if (currentAngle <= 0) {
      currentAngle   = 0;
      sweepDirection = 1;
      delay(200);
    }

    setAngle(currentAngle);
    delay(SWEEP_DELAY_MS);
    float dist = getDistance();

    Serial.print("[SWEEP]  Angle: ");
    Serial.print(currentAngle);
    Serial.print("°  |  Distance: ");
    Serial.print(dist, 1);
    Serial.print(" cm  |  BG: ");
    Serial.print(bgMap[angleToIndex(currentAngle)], 1);
    Serial.println(" cm");

    if (isIntrusion(currentAngle, dist)) {
      Serial.println("\n*** Intrusion detected — scanning for centroid ***");
      int centroid = findCentroid(currentAngle);
      if (centroid >= 0) {
        targetAngle = centroid;
        setAngle(targetAngle);
        resetPID();
        lastRescanTime = millis();
        Serial.print("*** Locked on at ");
        Serial.print(targetAngle);
        Serial.println("° — switching to TRACK mode ***\n");
        mode = TRACK;
      }
    }

  // --- TRACK MODE ---
  } else if (mode == TRACK) {
    // Blink red fast during tracking
    blinkLed(LED_RED, 150);
    digitalWrite(LED_GREEN, LOW);

    if (millis() - lastRescanTime >= RESCAN_INTERVAL_MS) {
      lastRescanTime = millis();

      int newCentroid = findCentroid(currentAngle);

      if (newCentroid < 0) {
        Serial.print("\n*** Object lost — waiting ");
        Serial.print(LOST_WAIT_MS / 1000.0, 1);
        Serial.println("s before resuming sweep ***\n");
        setLeds(false, false);
        delay(LOST_WAIT_MS);
        resetPID();
        mode = SWEEP;
        return;
      }

      targetAngle = newCentroid;
    }

    updatePID();
  }
}