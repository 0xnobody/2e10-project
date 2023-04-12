
#include <PID_v1.h>
#include <WiFiNINA.h>
#include <math.h>
#include <Arduino_LSM6DS3.h>

const int LED = 13;

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
double Kp = 0.1; // Kp needs to be low so it doesn't twerk
double Ki = 0.0; //  Ki needs to be very small
double Kd = 11; // Kd needs to be high because the buggy has to react very fast
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // Creating PID object

char ssid[] = "fuckthis";
char pass[] = "bullshit";

WiFiClient client;
WiFiServer server(5200);

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;

double error = 0;
double previousError = 0;
double integral = 0;
double derivative = 0;

double PID (double input, double setpoint) {
  error = setpoint - input;
  integral += error;
  derivative = error - previousError;
  previousError = error;

  return Kp * error + Kd * derivative + Ki * integral;
}

void setup() {
  Serial.begin(9600);

  WiFi.beginAP(ssid, pass);
  IPAddress ip = WiFi.localIP();
  Serial.println(ip);
  server.begin();

  Setpoint = -80.975; //-80.5; //angle where buggy balances itself

  // Turn the PID on.
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);

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

  // Set up encoders.
  pinMode(LENC, INPUT_PULLUP);
  pinMode(RENC, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LENC), left_wheel_ticks, RISING);
  attachInterrupt(digitalPinToInterrupt(RENC), right_wheel_ticks, RISING);
  
  srand(micros());

  if (!IMU.begin()) {
    Serial.println("Failed to init IMU");
  }

  Serial.println("Calibrating IMU... Don't move the buggy please!!!!!!!");

  const int n_samples = 250;

  noInterrupts();
  for (int i = 0; i < n_samples; i++) {
    float gyroX, gyroY, gyroZ;

    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    gyroBiasX -= gyroX;
    gyroBiasY -= gyroY;
    gyroBiasZ -= gyroZ;
  }
  interrupts();

  gyroBiasX /= n_samples;
  gyroBiasY /= n_samples;
  gyroBiasZ /= n_samples;

  Serial.println("...Done!");
}

// Calculate velocity from encoders
volatile long left_wheel_ticks_count = 0; // Keep track of the number of left wheel ticks
volatile long right_wheel_ticks_count = 0; // Keep track of the number of right wheel ticks
volatile long ticks_count = 0; // Average from both wheels
const double wheel_circ = PI * 0.065; // in meters: circumference
const double ticks_revolution = 8 * 120; // Ticks per wheel revolution = 8 ticks per motor rotation × 120 motor rotations per wheel revolution
volatile long old_ticks = 0;
volatile long d_ticks = 0;
double old_time = 0;
double d_time = 0;
double old_distance = 0;
double d_distance = 0;
double velocity = 0; // Buggy
double relativeV = 0; // Relative velocity
double object_velocity = 0; // Object

// Increment the number of ticks by 1 for both wheels
void left_wheel_ticks() {
  left_wheel_ticks_count++;
}

void right_wheel_ticks() {
  right_wheel_ticks_count++;
}

void calc_velocity() {
  // Update ticks
  ticks_count = (left_wheel_ticks_count + right_wheel_ticks_count) / 2;
  d_ticks = ticks_count - old_ticks;
  old_ticks = ticks_count;

  // Update time
  d_time = (millis() - old_time) / 1000.0d;
  old_time = millis();

  // Calculate velocity of buggy
  velocity = max(0, (wheel_circ * d_ticks) / (ticks_revolution * d_time));
}

bool keepDriving = true;
const int telemetryDelayUpdateMs = 100;

float Gx, Gy, Gz;
float Ax, Ay, Az;
float GzAngle = 0;
float AzSpeed = 0;

double currentAngle = 0;
int PID_Speed = 0;
double PID_Output = 0;
const double angleThreshold = 1;

float clampAngle(float angle) {
  if (angle > 360){
    return angle - 360;  
  } 
  else if (angle < -360){
    return angle + 360;  
  }
  return angle;
}

void loop() {
  static int prevTime = millis();
  int currTime = millis();
  int elapsedTimeMs = currTime - prevTime;
  prevTime = currTime;

  float prevAx = Ax, prevAy = Ay, prevAz = Az;

  IMU.readAcceleration(Ax, Ay, Az);
  IMU.readGyroscope(Gx, Gy, Gz);

  currentAngle = atan2(Ay, Az) * RAD_TO_DEG;
  PID_Output = PID(currentAngle, Setpoint);
  
  //Serial.println("Ax:" + String(Ax) + ",Ay:" + String(Ay) + ",Az:" + String(Az));

  Gz += gyroBiasZ;
  GzAngle += Gz * ((float)elapsedTimeMs / 1000) * 1.15;
  GzAngle = clampAngle(GzAngle);

  //Serial.println("AzSpeed:" + String(AzSpeed) + ",GzAngle:" + String(GzAngle));

  //Serial.println(String(Gx) + " " + String(Gy) + " " + String(Gz));
  //Serial.println(String("Travelling at angle ") + String(GzAngle) + String(" with speed ") + String(velocity));


  // If data is available, read it and see if we should keep driving or stop.
  // Update keepDriving based on this.
  client = server.available();
  if (client.connected()) {
    char c = client.read();
    if (c == 'g') {
      keepDriving = true;
    }
    else if (c == 's') {
      keepDriving = false;
    }
  }
  
  if (abs(Setpoint - currentAngle) > angleThreshold) {
    if (abs(PID_Output) < 1) {
      PID_Speed = 75 + (180 * abs(PID_Output));
    } else PID_Speed = 255;

    if (PID_Output > 0) {
      //moveForward(PID_Speed, PID_Speed);
    } else {
      //moveBackwards(PID_Speed, PID_Speed);
    }

  } else {
    PID_Speed = 0;
    stop();
  }

  static int telemetryUpdateTimeMs = 0;
  if (millis() - telemetryUpdateTimeMs > telemetryDelayUpdateMs) {
    telemetryUpdateTimeMs = millis();
    
    //calc_velocity();

    String angle_str = String("A:") + String((int)round(GzAngle * 1000000)) + "\n";
    server.write(angle_str.c_str());

    String tilt_str = String("T:") + String((int)round((currentAngle - Setpoint) * 1000000)) + "\n";
    server.write(tilt_str.c_str());

    String speed_str = String("S:") + String((int)round(((float)PID_Speed / 255) * 1000000)) + "\n";
    server.write(speed_str.c_str());

    server.write("PUSH");
  }

  //Serial.println(String(currentAngle) + " " + String(PID_Output) + " " + String(PID_Speed));
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
