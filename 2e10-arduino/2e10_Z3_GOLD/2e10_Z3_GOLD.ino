
#include <PID_v1.h>
#include <WiFiNINA.h>
#include <math.h>
#include <Arduino_LSM6DS3.h>

const int LED = 13;

#define PI 3.1415926535897932384626433832795

// LEFT MOTOR
const int LMOTOR1 = 10;
const int LMOTOR2 = 9;

// RIGHT MOTOR
const int RMOTOR1 = 12;
const int RMOTOR2 = 11;

// PID
double Input, Output, Setpoint;
double Kp = 0.1; // Kp needs to be low so it doesn't twerk
double Ki = 0.0; //  Ki needs to be very small
double Kd = 11; // Kd needs to be high because the buggy has to react very fast

char ssid[] = "2e10_Z3";
char pass[] = "gold_acrobat";

WiFiClient client;
WiFiServer server(5200);

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

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

  Setpoint = -80.975; //angle where buggy balances itself

  // Set up motors.
  pinMode(LMOTOR1, OUTPUT);
  pinMode(LMOTOR2, OUTPUT);

  pinMode(RMOTOR1, OUTPUT);
  pinMode(RMOTOR2, OUTPUT);
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

bool keepDriving = true;
const int telemetryDelayUpdateMs = 100;

float Gx, Gy, Gz;
float Ax, Ay, Az;
float GzAngle = 0, GyAngle = 0, GxAngle = 0;

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

  Gx += gyroBiasX;
  Gy += gyroBiasY;
  Gz += gyroBiasZ;
  GyAngle += Gy * ((float)elapsedTimeMs / 1000) * 1.15;
  GyAngle = clampAngle(GyAngle);

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
      moveForward(PID_Speed, PID_Speed);
    } else {
      moveBackwards(PID_Speed, PID_Speed);
    }

  } else {
    PID_Speed = 0;
    stop();
  }

  static int telemetryUpdateTimeMs = 0;
  if (millis() - telemetryUpdateTimeMs > telemetryDelayUpdateMs) {
    telemetryUpdateTimeMs = millis();
    
    String angle_str = String("A:") + String((int)round(GyAngle * 1000000)) + "\n";
    server.write(angle_str.c_str());

    String tilt_str = String("T:") + String((int)round((currentAngle - Setpoint) * 1000000)) + "\n";
    server.write(tilt_str.c_str());

    float realSpeed = (PID_Output > 0) ? ((float)PID_Speed / 255) : -((float)PID_Speed / 255);
    String speed_str = String("S:") + String((int)round(realSpeed * 1000000)) + "\n";
    server.write(speed_str.c_str());

    server.write("PUSH");
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
