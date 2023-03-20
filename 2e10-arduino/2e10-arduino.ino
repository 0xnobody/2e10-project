
#include <PID_v1.h>
#include <WiFiNINA.h>
#include <math.h>

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
double Kp = 15; // Kp needs to be higher than 10
double Ki = 0.035; //  Ki needs to be very small
double Kd = 1; // Kd keep as 0 because Kd is a derivative and will only work for steady signals so the rate of error is always changing
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // Creating PID object

char ssid[] = "fuckthis";
char pass[] = "bullshit";

WiFiClient client;
WiFiServer server(5200);

void setup() {
  Serial.begin(9600);

  WiFi.beginAP(ssid, pass);
  IPAddress ip = WiFi.localIP();
  Serial.println(ip);
  server.begin();

  Setpoint = 20; //buggy stops

  // Turn the PID on.
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0,255);

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
  attachInterrupt(digitalPinToInterrupt(LENC), left_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(RENC), right_wheel_pulse, RISING);
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
  return digitalRead(LEYE) == HIGH;
}
bool reyeStatus() {
  return digitalRead(REYE) == HIGH;
}

float distanceToObstacleCm() {
  // microseconds * toSecondsFactor * speedOfSound * toCmFactor * 0.5 (sound bounces)
  return (float)echoDurationUs * 0.000001 * 343 * 100 * 0.5;
}

// Calculate velocity from encoders
volatile long left_wheel_pulse_count = 0; // Keep track of the number of left wheel pulses
volatile long right_wheel_pulse_count = 0; // Keep track of the number of right wheel pulses
volatile long pulse_count = 0; // Average from both wheels
const int wheel_circ = PI * 0.065; // in meters: circumference
const int ticks_revolution = 8 * 120; // Ticks per wheel revolution = 8 ticks per motor rotation × 120 motor rotations per wheel revolution
volatile long old_ticks = 0;
volatile long d_ticks = 0;
volatile long old_time = 0;
volatile long d_time = 0;
volatile long velocity = 0;

// Increment the number of pulses by 1 for both wheels
void left_wheel_pulse() {
  left_wheel_pulse_count++;
}

void right_wheel_pulse() {
  right_wheel_pulse_count++;
}

void calc_velocity() {
  // Update ticks
  pulse_count = (left_wheel_pulse_count + right_wheel_pulse_count) / 2;
  d_ticks = pulse_count - old_ticks;
  old_ticks = pulse_count;

  // Update time
  d_time = (millis() - old_time) / 1000;
  old_time = millis();

  // Calculate velocity
  //velocity = (wheel_circ * d_ticks) / (ticks_revolution * d_time);
  velocity = ((wheel_circ / ticks_revolution) * d_ticks) / d_time;
}

bool obstacleDetected = false;
bool keepDriving = true;
const float obstacleStopDistanceCm = 10;
const int telemetryDelayUpdateMs = 500;

void loop() {
  // We need to pulse on each tick to have an up-to-date obstacle distance.
  pulse();
  
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

  Input = distanceToObstacleCm();

  calc_velocity();

  Serial.print("Left Ticks: ");
  Serial.println(left_wheel_pulse_count);
  Serial.print("Right Ticks: ");
  Serial.println(right_wheel_pulse_count);
  Serial.print("d Ticks: ");
  Serial.println(d_ticks);  
  Serial.print("d Time: ");
  Serial.println(d_time);  
  Serial.print("Velocity: ");
  Serial.println(velocity);
  delay(1000);
  
  /*
  static int telemetryUpdateTimeMs = 0;
  if (millis() - telemetryUpdateTimeMs > telemetryDelayUpdateMs) {
    telemetryUpdateTimeMs = millis();
    
    String dist_str = String("D:") + String(round(Input));
    server.write(dist_str.c_str());

    String speed_str = String("S:") + String(round(velocity)) + "\n";
    server.write(speed_str.c_str());
    
    // TODO: replace 420.0f with the object speed
    String obj_speed_str = String("OS:") + String(420.0f);
    server.write(obj_speed_str.c_str());
  }
  */

  // Check if an obstacle is detected. If so, we stop.
  // We also update obstacleDetected to true.
  if (distanceToObstacleCm() < obstacleStopDistanceCm) {
    stop(); 

    // If previously an obstacle was NOT detected, we inform the
    // server that one is detected.
    if (!obstacleDetected) {
      server.write("Obstacle detected!");
    }
    obstacleDetected = true;
    return;
  }
  
  // If we get here, there is no obstacle and we are clear to drive.
  obstacleDetected = false;

  // We only drive if the server has set keepDriving = true.
  if (!keepDriving) {
    stop();
    return;
  }

  myPID.Compute();

  //
  // Now we move the wheels based on the status of each eye.
  //
  
  if (leyeStatus() && reyeStatus()){
    moveForward(255-Output, 255-Output);
    return;
  }

  if (leyeStatus() && !reyeStatus()){
   moveForward((255-Output)/2, 255-Output);
   return;
  }

  if (!leyeStatus() && reyeStatus()){
   moveForward(255-Output, (255-Output)/2);
   return;
  }

  if (!leyeStatus() && !reyeStatus()){
   stop();
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
