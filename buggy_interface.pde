import processing.net.*;
import controlP5.*;
Client client;
String data;

ControlP5 p5;
color bgcol = color(40);
Button StartButton;

/*
  void Start() {
    client.write("g");
  }
  void Stop() {
    client.write("s");
  }
  */

PImage  img;

PImage ArrowRight,ArrowLeft,ArrowUp,ArrowDown;

int dist = 5, spd = 5, obj_spd = 5;


void setup() {
  size(1000, 700);
  
   img = loadImage("buggy.PNG");
   ArrowRight = loadImage("arrow.png");
   ArrowDown = loadImage("arrowdown.png");
   ArrowUp = loadImage("arrowup.png");
   ArrowLeft = loadImage("arrowleft.png");
   
  
 //  client = new Client(this, "192.168.4.1", 5200);
//  client.write("I am a new client");
      PFont pfont = createFont("Arial",20,true); // use true/false for smooth/no-smooth
  ControlFont font = new ControlFont(pfont,241);
    p5 = new ControlP5(this);
 Button startButton  = p5.addButton("Start")
  .setPosition(100, 100)
  .setSize(400, 40);
    
    
  p5.addButton("Stop")
  .setPosition(100, 200)
  .setSize(400, 40);
  
  p5.addButton("Right")
  .setPosition(350, 550)
  .setImage(ArrowRight);
  
  p5.addButton("Up")
  .setPosition(300, 500)
  .setImage(ArrowUp);
    
  p5.addButton("Left")
  .setPosition(250, 550)
  .setImage(ArrowLeft);
  
  p5.addButton("Down")
  .setPosition(300, 550)
  .setImage(ArrowDown);
    
    
  
 


}

void draw() {
    background(bgcol);
  image(img,600,50,300,300);

  /*
  data = client.readString();
  client.write("a");
  println(data);
  */
  textSize(32);
  text("Distance to obstacle: " + dist + " cm", 100, 300); 
  
  text("Current speed: " + spd + " cm", 100, 350); 
  
  text("Distance to obstacle: " + obj_spd + " cm", 100,400);
  
  
  

}


public void Start (boolean theValue) {
  //Start();
  //mycol = color(255,255,255);
  
}

public void Stop (boolean theValue) {
  
  //Start();
 // mycol = color(0,0,0);
  
}
