const int US_TRIG = 8;
const int US_ECHO = 2;
bool EchoInProgress = false;
long unsigned time = 0 ;

void TriggerEcho();

void EchoInterrupt();



void setup() {
  pinMode(US_TRIG, OUTPUT);
   pinMode(US_ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(US_ECHO),EchoInterrupt, CHANGE);




  
}
void EchoInterrupt() {

  //bool EchoInProgress = false;
  
  //int distance;
 // long duration;
  //duration = pulseIn( US_ECHO, HIGH );
  if(digitalRead(US_ECHO)==HIGH)
    time = millis () ;
    
  else {
    bool EchoInProgress = false;

  unsigned long duration = millis() - time;
  unsigned long distance = duration/58;
  Serial.println(distance);
  }
  

}

/*void EchoEndInterrupt() {
  bool EchoInProgress = false;

  unsigned long duration = millis() - time;
  unsigned long distance = duration/58;
  Serial.println(distance);
}*/

void TriggerEcho () {

  digitalWrite( US_TRIG, LOW );
  delayMicroseconds(2);

  digitalWrite( US_TRIG, HIGH );
  delayMicroseconds( 10 );
  digitalWrite( US_TRIG, LOW );

  bool EchoInProgress = true;
}

void loop() {

  if (!EchoInProgress)
    TriggerEcho();


}