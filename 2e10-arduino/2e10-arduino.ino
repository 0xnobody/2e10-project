
#include <PID_v1.h>
#include <WiFiNINA.h>

const int LED = 13;

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

//PID
double Input, Output, Setpoint;
double Kp = 15; // Kp needs to be higher than 10
double Ki = 0.001; //  Ki needs to be very small
double Kd = 2; // Kd keep as 0 because Kd is a derivative and will only work for steady signals so the rate of error is always changing
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // Creating PID object

char ssid[] = "fuckthis";
char pass[] = "bullshit";

WiFiClient client;
WiFiServer server(5200);

void setup() {
  Serial.begin(9600);

  WiFi.beginAP(ssid, pass);
  IPAddress ip = WiFi.localIP();
  server.begin();

  Setpoint = 20; //buggy stops

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0,255);

  // Set up input sensors.
  //
  pinMode( LEYE, INPUT );
  pinMode( REYE, INPUT );

  // Set up motors.
  //
  pinMode( LMOTOR1, OUTPUT );
  pinMode( LMOTOR2, OUTPUT );

  pinMode( RMOTOR1, OUTPUT );
  pinMode( RMOTOR2, OUTPUT );

  // Set up ultrasound.
  //
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(LED, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO), echoInterrupt, CHANGE);
}

void pulse() {
  static int lastTimePulsed = 0;
  static bool pulseSent = false;

  int timeSinceLastPulse = micros() - lastTimePulsed;

  // Check if we have waited long enough to trigger another pulse.
  //
  if (!pulseSent && timeSinceLastPulse > pulseDelayUs) {
    digitalWrite(TRIG, HIGH);
    lastTimePulsed = micros();
    pulseSent = true;
    return;
  }

  // Check if pulse duration elapsed.
  //
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

bool obstacleDetected = false;
bool keepDriving = true;
const float obstacleStopDistanceCm = 10;

void loop() {
  // We need to pulse on each tick to have an up-to-date obstacle distance.
  //
  pulse();
  
  // If data is available, read it and see if we should keep driving or stop.
  // Update keepDriving based on this.
  //
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

  // Check if an obstacle is detected. If so, we stop.
  // We also update obstacleDetected to true.
  //
  if (distanceToObstacleCm() < obstacleStopDistanceCm) {
    stop(); 

    // If previously an obstacle was NOT detected, we inform the
    // server that one is detected.
    //
    if (!obstacleDetected) {
      server.write("Obstacle detected!");
    }
    obstacleDetected = true;
    return;
  }
  
  // If we get here, there is no obstacle and we are clear to drive.
  //
  obstacleDetected = false;

  // We only drive if the server has set keepDriving = true.
  //
  if (!keepDriving) {
    stop();
    return;
  }

  myPID.Compute();
  Serial.print("Input: ");
  Serial.println(Input);
  Serial.print("Output: ");
  Serial.println(Output);
  
  //
  // Now we move the wheels based on the status of each eye.
  //
  
  if (leyeStatus() && reyeStatus()){
    moveForwards(255-Output, 255-Output);
    return;
  }

  if (leyeStatus() && !reyeStatus()){
   moveForwards(0, 255-Output);
   return;
  }

  if (!leyeStatus() && reyeStatus()){
   moveForwards(255-Output, 0);
   return;
  }

  if (!leyeStatus() && !reyeStatus()){
   stop();
  }
}

void stop() {
  moveForwards(0, 0);
}

void moveForwards(int leftPower, int rightPower) {
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
