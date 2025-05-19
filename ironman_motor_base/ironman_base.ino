#include <Arduino.h>

#define NUM_CH 4

// Motor control pins
const int motorPWM[NUM_CH] = {5, 6, 7, 8};
const int motorDirA[NUM_CH] = {22, 24, 26, 28};
const int motorDirB[NUM_CH] = {23, 25, 27, 29};

// Encoder pins
const int encoderA[NUM_CH] = {2, 3, 18, 19};   // Interrupt-capable
const int encoderB[NUM_CH] = {20, 21, 14, 15}; // Regular digital

// Ultrasonic sensor pins
#define FRONT_TRIG 30
#define FRONT_ECHO 31
#define LEFT_TRIG 32
#define LEFT_ECHO 33
#define RIGHT_TRIG 34
#define RIGHT_ECHO 35
#define REAR_TRIG 36
#define REAR_ECHO 37

#define OBSTACLE_THRESHOLD_CM 20

volatile long encoderTicks[NUM_CH] = {0};
float targetRPM[NUM_CH] = {0};
float currentRPM[NUM_CH] = {0};
float pwmOutput[NUM_CH] = {0};

float kp[NUM_CH] = {1.2, 1.2, 1.2, 1.2};
float ki[NUM_CH] = {0.5, 0.5, 0.5, 0.5};
float kd[NUM_CH] = {0.2, 0.2, 0.2, 0.2};

float errorSum[NUM_CH] = {0};
float lastError[NUM_CH] = {0};

const float wheelRadius = 0.097;
const float robotLength = 0.33;
const float robotWidth  = 0.29;

const int ticksPerRevolution = 330;
const unsigned long controlInterval = 20; // milliseconds
long lastTicks[NUM_CH] = {0};

// Motion state
bool motionActive = false;
long startTicks[NUM_CH];
long ticksTarget = 0;
float motionDirVX = 0, motionDirVY = 0, motionDirOmega = 0;
unsigned long lastControlTime = 0;

// Ultrasonic usage flag (false by default)
bool USE_ULTRASONIC = false;

// Motion command queue with wait time
struct MotionCommand {
  float distance_m;
  float speed_rpm;
  float dir_vx, dir_vy, dir_omega;
  unsigned long waitAfterMs;  // Wait time after motion in milliseconds
};
#define MAX_QUEUE 10
MotionCommand motionQueue[MAX_QUEUE];
int motionQueueSize = 0;
int currentMotionIndex = 0;

bool waitingAfterMotion = false;
unsigned long waitStartTime = 0;

// Encoder ISRs
void encoderISR0() { encoderTicks[0] += (digitalRead(encoderA[0]) == digitalRead(encoderB[0])) ? 1 : -1; }
void encoderISR1() { encoderTicks[1] += (digitalRead(encoderA[1]) == digitalRead(encoderB[1])) ? 1 : -1; }
void encoderISR2() { encoderTicks[2] += (digitalRead(encoderA[2]) == digitalRead(encoderB[2])) ? 1 : -1; }
void encoderISR3() { encoderTicks[3] += (digitalRead(encoderA[3]) == digitalRead(encoderB[3])) ? 1 : -1; }

// Ultrasonic distance read
long getDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout 30ms
  return duration * 0.034 / 2;
}

void queueMotion(float distance_m, float speed_rpm, float dir_vx, float dir_vy, float dir_omega, unsigned long waitMs = 0) {
  if (motionQueueSize < MAX_QUEUE) {
    motionQueue[motionQueueSize++] = { distance_m, speed_rpm, dir_vx, dir_vy, dir_omega, waitMs };
  }
}

void resetControlState() {
  for (int i = 0; i < NUM_CH; i++) {
    errorSum[i] = 0;
    lastError[i] = 0;
    lastTicks[i] = encoderTicks[i];
    pwmOutput[i] = 0;
    currentRPM[i] = 0;
  }
}

void setup() {
  for (int i = 0; i < NUM_CH; i++) {
    pinMode(motorPWM[i], OUTPUT);
    pinMode(motorDirA[i], OUTPUT);
    pinMode(motorDirB[i], OUTPUT);
    pinMode(encoderA[i], INPUT);
    pinMode(encoderB[i], INPUT);
  }

  attachInterrupt(digitalPinToInterrupt(encoderA[0]), encoderISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA[1]), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA[2]), encoderISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA[3]), encoderISR3, CHANGE);

  pinMode(FRONT_TRIG, OUTPUT); pinMode(FRONT_ECHO, INPUT);
  pinMode(LEFT_TRIG, OUTPUT); pinMode(LEFT_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT); pinMode(RIGHT_ECHO, INPUT);
  pinMode(REAR_TRIG, OUTPUT); pinMode(REAR_ECHO, INPUT);

  Serial.begin(9600);
  delay(2000);

  queueMotion(0.5, 50, -1, 0, 0, 5000); // BACKWARD
  queueMotion(0.5, 50, 1, 0, 0, 5000);  // FORWARD
  // Move left 0.5 meters at 50 RPM, wait 5 seconds
  queueMotion(0.5, 50, 0, -1, 0, 5000);
  // Move right 0.5 meters at 50 RPM, wait 5 seconds
  queueMotion(0.5, 50, 0, 1, 0, 5000);
  // Rotate in place 360 degrees at 50 RPM, wait 5 seconds
  float radius = (robotLength + robotWidth) / 2.0;
  float rotateDistance = 2 * PI * radius;  // Distance each wheel travels for 360° turn
  queueMotion(rotateDistance, 50, 0, 0, 1, 5000);

  if (motionQueueSize > 0) {
    MotionCommand& cmd = motionQueue[0];
    startMotion(cmd.distance_m, cmd.speed_rpm, cmd.dir_vx, cmd.dir_vy, cmd.dir_omega);
  }
}

void startMotion(float distance_m, float speed_rpm, float dir_vx, float dir_vy, float dir_omega) {
  float revs = distance_m / (2 * PI * wheelRadius);
  ticksTarget = revs * ticksPerRevolution;

  for (int i = 0; i < NUM_CH; i++) startTicks[i] = encoderTicks[i];

  resetControlState();

  mecanumKinematics(dir_vx * speed_rpm, dir_vy * speed_rpm, dir_omega * speed_rpm);

  motionDirVX = dir_vx;
  motionDirVY = dir_vy;
  motionDirOmega = dir_omega;
  motionActive = true;
  lastControlTime = millis();

  Serial.print("Starting motion. Distance: ");
  Serial.print(distance_m);
  Serial.print(" m, Speed: ");
  Serial.print(speed_rpm);
  Serial.println(" rpm");
}

void stopAllMotors() {
  for (int i = 0; i < NUM_CH; i++) {
    targetRPM[i] = 0;
    setMotor(i, 0, 0, 0, 0);
  }
  motionActive = false;

  unsigned long waitMs = motionQueue[currentMotionIndex].waitAfterMs;
  if (waitMs > 0) {
    waitingAfterMotion = true;
    waitStartTime = millis();
    Serial.print("Waiting for ");
    Serial.print(waitMs);
    Serial.println(" ms after motion.");
  } else {
    waitingAfterMotion = false;
    advanceToNextMotion();
  }
}

void advanceToNextMotion() {
  currentMotionIndex++;
  if (currentMotionIndex < motionQueueSize) {
    MotionCommand& cmd = motionQueue[currentMotionIndex];
    startMotion(cmd.distance_m, cmd.speed_rpm, cmd.dir_vx, cmd.dir_vy, cmd.dir_omega);
  } else {
    Serial.println("All motions complete.");
  }
}

void mecanumKinematics(float vx, float vy, float omega) {
  float L = robotLength;
  float W = robotWidth;
  float R = wheelRadius;

  float wheelRPM[NUM_CH];
  wheelRPM[0] = (1 / R) * (vx - vy - (L + W) * omega);
  wheelRPM[1] = (1 / R) * (vx + vy + (L + W) * omega);
  wheelRPM[2] = (1 / R) * (vx + vy - (L + W) * omega);
  wheelRPM[3] = (1 / R) * (vx - vy + (L + W) * omega);

  for (int i = 0; i < NUM_CH; i++) targetRPM[i] = wheelRPM[i];
}

void setMotor(int i, float pwmVal, float dir_vx, float dir_vy, float dir_omega) {
  float direction = 0;
  switch (i) {
    case 0: direction = dir_vx - dir_vy - (robotLength + robotWidth) * dir_omega; break;
    case 1: direction = dir_vx + dir_vy + (robotLength + robotWidth) * dir_omega; break;
    case 2: direction = dir_vx + dir_vy - (robotLength + robotWidth) * dir_omega; break;
    case 3: direction = dir_vx - dir_vy + (robotLength + robotWidth) * dir_omega; break;
  }

  if (direction >= 0) {
    digitalWrite(motorDirA[i], HIGH);
    digitalWrite(motorDirB[i], LOW);
  } else {
    digitalWrite(motorDirA[i], LOW);
    digitalWrite(motorDirB[i], HIGH);
    pwmVal = -pwmVal;
  }

  analogWrite(motorPWM[i], constrain(pwmVal, 0, 255));
}

void controlMotors(float dir_vx, float dir_vy, float dir_omega) {
  for (int i = 0; i < NUM_CH; i++) {
    long ticksNow = encoderTicks[i];
    long ticksDelta = ticksNow - lastTicks[i];
    lastTicks[i] = ticksNow;

    currentRPM[i] = (ticksDelta * 60000.0 / controlInterval) / ticksPerRevolution;

    float error = targetRPM[i] - currentRPM[i];
    errorSum[i] += error;
    float dError = error - lastError[i];
    lastError[i] = error;

    pwmOutput[i] = kp[i] * error + ki[i] * errorSum[i] + kd[i] * dError;

    setMotor(i, pwmOutput[i], dir_vx, dir_vy, dir_omega);
  }
}

void loop() {
  if (waitingAfterMotion) {
    if (millis() - waitStartTime >= motionQueue[currentMotionIndex].waitAfterMs) {
      waitingAfterMotion = false;
      advanceToNextMotion();
    }
    return;
  }

  if (motionActive) {
    if (USE_ULTRASONIC) {
      long dFront = getDistanceCM(FRONT_TRIG, FRONT_ECHO);
      long dLeft = getDistanceCM(LEFT_TRIG, LEFT_ECHO);
      long dRight = getDistanceCM(RIGHT_TRIG, RIGHT_ECHO);
      long dRear = getDistanceCM(REAR_TRIG, REAR_ECHO);

      if ((motionDirVX > 0 && dFront < OBSTACLE_THRESHOLD_CM) ||
          (motionDirVX < 0 && dRear < OBSTACLE_THRESHOLD_CM)  ||
          (motionDirVY > 0 && dLeft < OBSTACLE_THRESHOLD_CM)  ||
          (motionDirVY < 0 && dRight < OBSTACLE_THRESHOLD_CM)) {
        Serial.println("Obstacle detected! Stopping motion.");
        stopAllMotors();
        delay(1000);
        return;
      }
    }

    bool done = true;
    for (int i = 0; i < NUM_CH; i++) {
      if (abs(encoderTicks[i] - startTicks[i]) < ticksTarget) {
        done = false;
        break;
      }
    }

    if (done) {
      stopAllMotors();
    }

    unsigned long now = millis();
    if (now - lastControlTime >= controlInterval) {
      lastControlTime = now;
      controlMotors(motionDirVX, motionDirVY, motionDirOmega);
    }
  }
}