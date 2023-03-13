import processing.net.*;

Client client;
String data;

color red = color(255,0,0);
color green = color(0,255,0);
color blue = color(0,0,255);
color white = color(255,255,255);

PImage img;

class Button {
  
  float x,y,w,h;
  boolean buttonOn = true;
  String text;
  float fontSize = 32;
  
  Button( float xnew, float ynew, String name) {
    x = xnew;
    y = ynew;
    w=100;
    h=40;
    text = name;
  }
  
  Button( float xnew, float ynew, float wnew, float hnew, String name) {
    x = xnew;
    y = ynew;
    w = wnew;
    h = hnew;
    text = name;
  }
  
  void update() {
    if (buttonOn) {
      fill(green);
    }
    else {
      fill(red);  
    }
    
    rect(x, y, w, h);
    fill(0,0,0);
    textSize(fontSize);
    text(text,x,y+30);
  }
  
  void Start() {
    client.write("g");
  }
  void Stop() {
    client.write("s");
  }
  
  void click() {
    if (mouseX >= x && mouseY >= y && mouseX <= x+w && mouseY <= y+h ){
      buttonOn = !buttonOn;
      
      if (buttonOn) {
        Start();
      }
      else{
        Stop();
      }
    } 
  }
  
};

class StartButton extends Button {
  
  StartButton( float xnew, float ynew, String name) {
    super(xnew, ynew, name);
  }
  void Start() {
    client.write("g");
  }
  void Stop() {
    client.write("s");
  }
}
  
Button startButton = new StartButton(100,100,"Start");

void setup() {
  size(1000, 700);
  
  img = loadImage("buggy.jpg");
  
  client = new Client(this, "192.168.4.1", 5200);
}

void draw() {
  
  image(img,0,0,width,height);
  
  startButton.update();
  
  data = client.readString();
  if (data != null) {
     println(data);
  }
}

void mouseClicked() {
  startButton.click();
}
