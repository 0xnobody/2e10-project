import processing.net.*;
import controlP5.*;
Client client;
String data;

ControlP5 p5;
color bgcol = color(40);
Button StartButton;

PImage img, buggy_model;

double dist = 5, spd = 5, obj_spd = 5;

float tiltAngle = 0;
float headingAngle = 0;
float speed = 0;

PathWindow wnd = null;

void setup() {
  size(1000, 700);

  String[] args = { "Hello!" };
  wnd = new PathWindow();
  PApplet.runSketch(args, wnd);
  
  img = loadImage("buggy.PNG");
  buggy_model = loadImage("buggyspr.png");

  client = new Client(this, "192.168.4.1", 5200);
  client.write("I am a new client");

  p5 = new ControlP5(this);
  p5.addButton("Start")
    .setPosition(100, 100)
    .setSize(400, 40);

  p5.addButton("Stop")
    .setPosition(100, 200)
    .setSize(400, 40);
}

void draw() {
  background(bgcol);

  data = client.readString();

  if (data != null) {
    for (String msg : data.split("\n")) {
      println(msg);
      if (!msg.isEmpty()) {
        processMessage(msg.trim());
      }
    }
  }

  textSize(32);

  text("Rotation: " + obj_spd + " degrees", 600, 100);

  //image(img,600,50,300,300);

  translate(800, 500);
  rotate(buggy_rotation  * PI / 180);

  //translate(-width/2, -height/2);
  imageMode(CENTER);
  image(buggy_model, 0, 0, 500, 500);


  rotate(-buggy_rotation  * PI / 180);
  translate(-800, -500);
}

void processMessage(String message) {
  println(message);
  if (message.startsWith("A:")) {
    try {
      headingAngle = (double)Integer.parseInt(message.substring(2)) / 1000000;
    }
    catch (NumberFormatException e) {}
  } else if (message.startsWith("T:")) {
    try {
      tiltAngle = (double)Integer.parseInt(message.substring(2)) / 1000000;
    }
    catch (NumberFormatException e) {}
  } else if (message.startsWith("S:")) {
    try {
      speed = (double)Integer.parseInt(message.substring(2)) / 1000000;
    }
    catch (NumberFormatException e) {}
  } else if (message.startsWith("PUSH")) {
    wnd.recordMovement(new PVector(cos(headingAngle), sin(headingAngle)), speed);
  } else {
    println(data);
  }
}

public void Start() {
  client.write("g");
}

public void Stop() {
  client.write("s");
}

public class PathWindow extends PApplet {
  const PVector wndDimensions = new PVector(500, 500);
  const float scale = 0.01;
  
  PVector currentPosition = wndDimensions / 2;
  float lastRecordedTime = 0;
  
  public void settings() {
    size(wndDimensions.x, wndDimensions.y);
  }
  
  public void draw() {
    background(255);
    fill(0);
    ellipse(100, 50, 10, 10);
  }
  
  public void recordMovement(PVector direction, float magnitude) {
    var currTimeMs = millis();
    var elapsedTimeMs = currTimeMs - lastRecordedTime;
    
    var origin = currentPosition;
    var dest = origin + direction * (elapsedTimeMs * scale) * magnitude;
    
    stroke(255);
    line(origin.x, origin.y, dest.x, dest.y);
    
    currentPosition = dest;
    lastRecordedTime = currTimeMs;
  }
}
