
#include <PID_v1.h>
#include <WiFiNINA.h>
#include <math.h>
#include <Arduino_LSM6DS3.h>

const int LED = 13;

struct TreeNode {
  TreeNode* parent = nullptr;
  TreeNode* lhs = nullptr;
  TreeNode* rhs = nullptr;
  bool explored = false;

  TreeNode* forkLeft() {
    auto node = new TreeNode();
    node->parent = this;
    lhs = node;
    return lhs;
  }

  TreeNode* forkRight() {
    auto node = new TreeNode();
    node->parent = this;
    rhs = node;
    return rhs;
  }

  void explore() {
    explored = true;
    if (parent && parent != this) {
      if (parent->lhs->explored && parent->rhs->explored) {
        parent->explore();
      }
    }
  }
};

TreeNode* rootNode = nullptr;
TreeNode* currentNode = nullptr;

enum class EBuggyStatus {
  GoingStraight,
  TurningLeft,
  TurningRight,
  TurningAround,
  Idle
};

EBuggyStatus currentStatus = EBuggyStatus::GoingStraight;

#define PI 3.1415926535897932384626433832795

// IR SENSORS
const int LEYE = 21; //left sensor
const int REYE = 20; //right sensor

// LEFT MOTOR
const int LMOTOR1 = 10;
const int LMOTOR2 = 9;

// RIGHT MOTOR
const int RMOTOR1 = 12;
const int RMOTOR2 = 11;

// ULTRASOUND
const int ECHO = 15;
const int TRIG = 14;

const int pulseDurationUs = 50;
const int pulseDelayUs = 100000;

// ENCODERS
const int LENC = 2; //left encoder
const int RENC = 3; //right encoder

// PID
double Input, Output, Setpoint;
double Kp = 15; // Kp needs to be higher than 10
double Ki = 0.035; //  Ki needs to be very small
double Kd = 1; // Kd keep as 0 because Kd is a derivative and will only work for steady signals so the rate of error is always changing
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // Creating PID object

char ssid[] = "fuckthis";
char pass[] = "bullshit";

WiFiClient client;
WiFiServer server(5200);

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
void setup() {
  Serial.begin(9600);

  //WiFi.beginAP(ssid, pass);
  //IPAddress ip = WiFi.localIP();
  //Serial.println(ip);
  //server.begin();

  //Setpoint = 20; //buggy stops

  // Turn the PID on.
  //myPID.SetMode(AUTOMATIC);
  //myPID.SetOutputLimits(0,255);

  // Set up input sensors.
  pinMode(LEYE, INPUT);
  pinMode(REYE, INPUT);

  // Set up motors.
  pinMode(LMOTOR1, OUTPUT);
  pinMode(LMOTOR2, OUTPUT);

  pinMode(RMOTOR1, OUTPUT);
  pinMode(RMOTOR2, OUTPUT);

  // Set up ultrasound.
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(LED, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO), echoInterrupt, CHANGE);

  // Set up encoders.
  pinMode(LENC, INPUT_PULLUP);
  pinMode(RENC, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LENC), left_wheel_ticks, RISING);
  attachInterrupt(digitalPinToInterrupt(RENC), right_wheel_ticks, RISING);

  rootNode = new TreeNode();
  rootNode->parent = rootNode;
  currentNode = rootNode;
  
  srand(micros());

  if (!IMU.begin()) {
    Serial.println("Failed to init IMU");
  }

  Serial.println("Calibrating IMU... Don't move the buggy please!!!!!!!");

  int calibrationStartMillis = millis();
  int n_samples = 0;
  while (millis() - calibrationStartMillis < 3000) {
      if (IMU.gyroscopeAvailable()) {
        float gyroX, gyroY, gyroZ;
        IMU.readGyroscope(gyroX, gyroY, gyroZ);

        gyroBiasX += gyroX;
        gyroBiasY += gyroY;
        gyroBiasZ += gyroZ;

        n_samples++;
      }
  }

  gyroBiasX /= n_samples;
  gyroBiasY /= n_samples;
  gyroBiasZ /= n_samples;

  Serial.println("...Done!");
}

void pulse() {
  static int lastTimePulsed = 0;
  static bool pulseSent = false;

  int timeSinceLastPulse = micros() - lastTimePulsed;

  // Check if we have waited long enough to trigger another pulse.
  if (!pulseSent && timeSinceLastPulse > pulseDelayUs) {
    digitalWrite(TRIG, HIGH);
    lastTimePulsed = micros();
    pulseSent = true;
    return;
  }

  // Check if pulse duration elapsed.
  if (pulseSent && timeSinceLastPulse > pulseDurationUs) {
    digitalWrite(TRIG, LOW);
    pulseSent = false;
  }
}

volatile int echoStartUs = 0;
volatile int echoEndUs = 0;
volatile int echoDurationUs = 0;
void echoInterrupt() {
  bool state = digitalRead(ECHO) == HIGH;
  if (state) {
    echoEndUs = 0;
    echoStartUs = micros();
  }
  else {
    echoEndUs = micros();
    echoDurationUs = max(0, echoEndUs - echoStartUs);
  }
}

bool leyeStatus() {
  return digitalRead(LEYE) != HIGH;
}
bool reyeStatus() {
  return digitalRead(REYE) != HIGH;
}

double distanceToObstacleCm() {
  // microseconds * toSecondsFactor * speedOfSound * toCmFactor * 0.5 (sound bounces)
  return (float)echoDurationUs * 0.000001 * 343 * 100 * 0.5;
}

// Calculate velocity from encoders
volatile long left_wheel_ticks_count = 0; // Keep track of the number of left wheel ticks
volatile long right_wheel_ticks_count = 0; // Keep track of the number of right wheel ticks

// Increment the number of ticks by 1 for both wheels
void left_wheel_ticks() {
  left_wheel_ticks_count++;
}

void right_wheel_ticks() {
  right_wheel_ticks_count++;
}

bool obstacleDetected = false;
bool keepDriving = true;
const float obstacleStopDistanceCm = 10;
const int telemetryDelayUpdateMs = 100;

float Gx, Gy, Gz;
void loop() {
  pulse();
  Serial.println(distanceToObstacleCm());

  static int lastGyroReadMillis = 0;
  if (IMU.gyroscopeAvailable()) {
    auto dt = millis() - lastGyroReadMillis;
    if (dt > 20) {
      float gyroX, gyroY, gyroZ;
      IMU.readGyroscope(gyroX, gyroY, gyroZ);

      lastGyroReadMillis = millis();

      float dt_sec = ((float)(dt + 3) / 1000);
      Gx += (gyroX - gyroBiasX) * dt_sec;
      Gy += (gyroY - gyroBiasY) * dt_sec;
      Gz += (gyroZ - gyroBiasZ) * dt_sec;

      Serial.print("X: ");
      Serial.print(Gx);
      Serial.print(" Y: ");
      Serial.print(Gy);
      Serial.print(" Z: ");
      Serial.print(Gz);
      Serial.println();
    }
  }

  static float turnGyroX, turnGyroY, turnGyroZ;

  if (currentStatus == EBuggyStatus::TurningLeft) {
    if (Gz - turnGyroZ >= 90) {
      stop();
      currentStatus = EBuggyStatus::GoingStraight;
      return;
    }
    turnLeft(80);
    return;
  }
  else if (currentStatus == EBuggyStatus::TurningRight) {
    if (Gz - turnGyroZ <= 90) {
      stop();
      currentStatus = EBuggyStatus::GoingStraight;
      return;
    }
    turnRight(80);
    return;
  }
  else if (currentStatus == EBuggyStatus::TurningAround) {
    if (Gz - turnGyroZ >= 360) {
      stop();
      currentStatus = EBuggyStatus::GoingStraight;
      return;
    }
    turnLeft(80);
  }
  else if (currentStatus == EBuggyStatus::GoingStraight) {    
    if (distanceToObstacleCm() < 10) {
      currentNode->explore();
      currentStatus = EBuggyStatus::TurningAround;
      Serial.print("Turning around, obstacle within ");
      Serial.println(distanceToObstacleCm());
    }
    
    static bool atBranch = false;
    static int correctionStartMillis = 0;
    if (atBranch) {      
      if (millis() - correctionStartMillis < 850) {
        moveForward(90, 90);
      }
      else {
        stop();
        // We now have 2 branches we can take - left or right.
        // Decide by random.
        //
        bool goLeft = true; //rand() % 2;
        if (goLeft) {
          currentStatus = EBuggyStatus::TurningLeft;
          currentNode = currentNode->forkLeft();
        } else {
          currentStatus = EBuggyStatus::TurningRight;
          currentNode = currentNode->forkRight();
        }

        turnGyroZ = Gz;
        atBranch = false;
      }
    }

    if (leyeStatus() && reyeStatus()){
      moveForward(90, 90);
      return;
    }

    if (leyeStatus() && !reyeStatus()){
      moveForward(60, 90);
      return;
    }

    if (!leyeStatus() && reyeStatus()){
      moveForward(90, 60);
      return;
    }
    
    // If both the eyes see black, we are at a branch.
    //
    if (!atBranch && !leyeStatus() && !reyeStatus()){
      atBranch = true;
      correctionStartMillis = millis();
    }
  }
}

void stop() {
  moveForward(0, 0);
}

void moveForward(int leftPower, int rightPower) {
  analogWrite(LMOTOR1, leftPower);
  analogWrite(LMOTOR2, 0);
  analogWrite(RMOTOR1, rightPower);
  analogWrite(RMOTOR2, 0);
}

void moveBackwards(int leftPower, int rightPower) {
  analogWrite(LMOTOR1, 0);
  analogWrite(LMOTOR2, leftPower);
  analogWrite(RMOTOR1, 0);
  analogWrite(RMOTOR2, rightPower);
}

void turnRight(int power) {
  analogWrite(LMOTOR1, 0);
  analogWrite(LMOTOR2, power);
  analogWrite(RMOTOR1, power);
  analogWrite(RMOTOR2, 0);
}

void turnLeft(int power) {
  analogWrite(LMOTOR1, power);
  analogWrite(LMOTOR2, 0);
  analogWrite(RMOTOR1, 0);
  analogWrite(RMOTOR2, power);
}