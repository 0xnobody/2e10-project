import processing.net.*;
import controlP5.*;
Client client;
String data;

ControlP5 p5;
color bgcol = color(40);
Button StartButton;


PImage  img, buggy_model;

PImage ArrowRight,ArrowLeft,ArrowUp,ArrowDown;

//String dist = " ", spd = " ", obj_spd = " ";


int dist = 5, spd = 5, obj_spd = 5;
float buggy_rotation = 0;


void setup() {
  size(1000, 700);
  
   img = loadImage("buggy.PNG");
   buggy_model = loadImage("buggyspr.png");
   ArrowRight = loadImage("arrow.png");
   ArrowDown = loadImage("arrowdown.png");
   ArrowUp = loadImage("arrowup.png");
   ArrowLeft = loadImage("arrowleft.png");
   
  
  client = new Client(this, "192.168.4.1", 5200);
  client.write("I am a new client");
 
  
    p5 = new ControlP5(this);
p5.addButton("Start")
  .setPosition(100, 100)
  .setSize(400, 40);
    
    
  p5.addButton("Stop")
  .setPosition(100, 200)
  .setSize(400, 40);
  /*
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
    
    */
  
 


}

void draw() {
    background(bgcol);
   



  
    
  if (data != null) {
    for (String msg : data.split("\n")) {
      if (!msg.isEmpty()) {
        processMessage(msg.trim());
      }
    }
  }

  textSize(32);
  text("Distance to obstacle: " + dist + " cm", 100, 300);
  text("Current speed: " + spd + " km/h", 100, 350);
  text("Object speed: " + obj_spd + " km/h", 100, 400);
  
  text("Rotation: " + obj_spd + " degrees", 600, 100);
  
  //image(img,600,50,300,300);
  
translate(800, 500);
  rotate(buggy_rotation  * PI / 180);
  
//translate(-width/2, -height/2);
imageMode(CENTER);
 image(buggy_model,0,0,500,500);


rotate(-buggy_rotation  * PI / 180);
translate(-800, -500);

}

void processMessage(String message) {
  if (message.startsWith("D:")) {
    try {
      dist = (int)Float.parseFloat(message.substring(2));
    }
    catch (NumberFormatException e) {
    }
  } else if (message.startsWith("S:")) {
    try {
      spd = (int)Float.parseFloat(message.substring(2));
    }
    catch (NumberFormatException e) {
    }
  } else if (message.startsWith("OS:")) {
    try {
      obj_spd = (int)Float.parseFloat(message.substring(3));
    }
    catch (NumberFormatException e) {
    }
  } else if (message.startsWith("R:")) {
    try {
      buggy_rotation = (int)Float.parseFloat(message.substring(2));
    }
    catch (NumberFormatException e) {
    }
  } else {
    println(data);
  }
}

public void Start (boolean theValue) {
  client.write("g");
  
}

public void Stop (boolean theValue) {
  
  client.write("s");
  
}
/*
public void Up (boolean theValue) {
  
  //client.write("f");
  
}
public void Down (boolean theValue) {
  
  //client.write("b");
  
}

public void Left (boolean theValue) {
  
  //client.write("l");
  
}

public void Right (boolean theValue) {
  
  //client.write("r");
  
}
*/
