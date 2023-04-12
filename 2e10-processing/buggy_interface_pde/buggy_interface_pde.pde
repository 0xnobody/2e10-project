import processing.net.*;
import controlP5.*;
import java.util.concurrent.locks.*;

Client client;
String data;

ControlP5 p5;
color bgcol = color(40);
Button StartButton;

PImage img, buggy_model;

double dist = 5, spd = 5, obj_spd = 5;

float tiltAngle = 0;
float prevHeadingAngle = 0;
float headingAngle = 0;
float speed = 0;

PathWindow wnd = null;

void setup() {
  size(1000, 700);

  String[] args = { "Hello!" };
  wnd = new PathWindow(); //<>//
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
      //println(msg);
      if (!msg.isEmpty()) {
        processMessage(msg.trim());
      }
    }
  }

  textSize(32);

  text("Rotation: " + obj_spd + " degrees", 600, 100);

  //image(img,600,50,300,300);

  translate(800, 500);
  rotate(tiltAngle  * PI / 180);

  //translate(-width/2, -height/2);
  imageMode(CENTER);
  image(buggy_model, 0, 0, 500, 500);


  rotate(-tiltAngle  * PI / 180);
  translate(-800, -500);
}

void processMessage(String message) {
  //println(message);
  if (message.startsWith("A:")) {
    try {
      headingAngle = (float)Integer.parseInt(message.substring(2)) / 1000000;
    }
    catch (NumberFormatException e) {}
  } else if (message.startsWith("T:")) {
    try {
      tiltAngle = (float)Integer.parseInt(message.substring(2)) / 1000000;
    }
    catch (NumberFormatException e) {}
  } else if (message.startsWith("S:")) {
    try {
      speed = (float)Integer.parseInt(message.substring(2)) / 1000000;
    }
    catch (NumberFormatException e) {}
  } else if (message.startsWith("PUSH")) {
    wnd.recordMovement(new PVector(cos((float)Math.toRadians(headingAngle)), sin((float)Math.toRadians(headingAngle))), speed);
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

class LineInfo {
  public PVector start;
  public PVector end;
  
  public LineInfo(PVector s, PVector e)
  {
    this.start = s;
    this.end = e;
  }
};

public class PathWindow extends PApplet {
  PVector wndDimensions = new PVector(500, 500);
  float scale = 0.01;
  
  PVector currentPosition = new PVector(wndDimensions.x / 2, wndDimensions.y / 2);
  float lastRecordedTime = 0;
  
  Lock mutex = new ReentrantLock();
  ArrayList<LineInfo> path = new ArrayList<LineInfo>();
  
  public void settings() {
    size((int)wndDimensions.x, (int)wndDimensions.y);
  }
  
  public void draw() {
    background(255);
    fill(0);
    
    var center = new PVector(wndDimensions.x / 2, wndDimensions.y / 2);
    
    ellipse(center.x, center.y, 10, 10); //<>//

    mutex.lock();
    for (LineInfo line : path) {
      stroke(0);
      strokeWeight(2);
      line(line.start.x, line.start.y, line.end.x, line.end.y);
    }
    mutex.unlock();
  }
  
  public void recordMovement(PVector direction, float magnitude) {
    var currTimeMs = millis();
    var elapsedTimeMs = currTimeMs - lastRecordedTime;
    
    var origin = currentPosition;
    var dest = PVector.add(origin, PVector.mult(direction, elapsedTimeMs * scale * magnitude));
    
    //println("Add path from x:" + origin.x + " y:" + origin.y + " -> x:" + dest.x + " y:" + dest.y);
    
    mutex.lock()
    path.add(new LineInfo(origin, dest));
    mutex.unlock();
    
    currentPosition = dest;
    lastRecordedTime = currTimeMs;
  }
}
