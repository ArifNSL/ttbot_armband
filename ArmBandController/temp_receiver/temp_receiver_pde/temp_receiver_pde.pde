import peasy.PeasyCam;
import processing.core.PVector;
import processing.serial.*;

Serial myPort;
String val;

float angleX = 0;
float angleY = 0;

// Right arm angles
float angleX1 = 0, angleY1 = 0, angleZ1 = 0; // Right Wrist
float angleX2 = 0, angleY2 = 0, angleZ2 = 0; // Right Elbow
float angleX3 = 0, angleY3 = 0, angleZ3 = 0; // Right Shoulder

// Left arm angles
float angleX4 = 0, angleY4 = 0, angleZ4 = 0; // Left Wrist
float angleX5 = 0, angleY5 = 0, angleZ5 = 0; // Left Elbow
float angleX6 = 0, angleY6 = 0, angleZ6 = 0; // Left Shoulder

float angleY9 = 0; // Body rotation

PeasyCam cam;

void setup() {
  size(800, 600, P3D);
  noStroke();
  
  cam = new PeasyCam(this, 500);
  
  // Initialize Serial communication
  String portName = "/dev/ttyUSB0";
  myPort = new Serial(this, portName, 115200);
  myPort.bufferUntil('\n');
}

void draw() {
  background(255);
  lights();
  
  rotateX(angleX);
  rotateY(angleY);
  
  // Rotate entire scene based on body rotation
  rotateY(radians(angleY9));
  
  // Draw main box (body)
  pushMatrix();
  fill(135, 206, 250);
  box(50, 150, 20);
  
  // Draw right arm
  pushMatrix();
  translate(-25, -60, 0);
  drawArm(angleX3, angleY3, angleZ3, angleX2, angleY2, angleZ2, angleX1, angleY1, angleZ1, 
          color(255, 0, 0), color(255, 102, 102), color(0, 255, 255), 
          "Right Shoulder", "Right Elbow", "Right Wrist");
  popMatrix();
  
  // Draw left arm
  pushMatrix();
  translate(25, -60, 0);
  drawArm(angleX6, angleY6, angleZ6, angleX5, angleY5, angleZ5, angleX4, angleY4, angleZ4, 
          color(0, 0, 255), color(144, 238, 144), color(0, 255, 0), 
          "Left Shoulder", "Left Elbow", "Left Wrist");
  popMatrix();
  
  popMatrix();
}

void drawArm(float shoulderX, float shoulderY, float shoulderZ,
             float elbowX, float elbowY, float elbowZ,
             float wristX, float wristY, float wristZ,
             color shoulderColor, color elbowColor, color wristColor,
             String shoulderLabel, String elbowLabel, String wristLabel) {
  
  // Shoulder
  pushMatrix();
  rotateZ(radians(shoulderZ));
  rotateX(radians(shoulderX));
  rotateY(radians(shoulderY));
  fill(shoulderColor);
  box(20, 40, 20);
  drawLabel(shoulderLabel);
  
  // Upper arm
  translate(0, 30, 0);
  fill(200);
  box(10, 60, 10);
  
  // Elbow
  translate(0, 30, 0);
  rotateX(radians(elbowX));
  rotateY(radians(elbowY));
  rotateZ(radians(elbowZ));
  fill(elbowColor);
  box(15, 30, 15);
  drawLabel(elbowLabel);
  
  // Forearm
  translate(0, 25, 0);
  fill(200);
  box(8, 50, 8);
  
  // Wrist
  translate(0, 25, 0);
  rotateX(radians(wristX));
  rotateY(radians(wristY));
  rotateZ(radians(wristZ));
  fill(wristColor);
  box(12, 20, 12);
  drawLabel(wristLabel);
  
  popMatrix();
}

void drawLabel(String label) {
  pushMatrix();
  fill(0);
  textSize(12);
  textAlign(CENTER, CENTER);
  translate(0, 30, 0);
  rotateX(-PI/2);
  text(label, 0, 0);
  popMatrix();
}

void serialEvent(Serial myPort) {
  val = myPort.readStringUntil('\n');
  if (val != null) {
    val = trim(val);
    String[] data = split(val, '\t');
    if (data.length == 31) { // Ensure we have all expected values
      // Right arm
      angleZ1 = float(data[0]); // yaw1
      angleY1 = 360-float(data[1]); // pitch1
      angleX1 = float(data[2]); // roll1
      
      angleZ2 = float(data[3]); // yaw2
      angleY2 = float(data[4]); // pitch2
      angleX2 = float(data[5]); // roll2
      
      angleZ3 = float(data[6]); // yaw3
      angleY3 = float(data[7]); // pitch3
      angleX3 = float(data[8]); // roll3
      
      // Left arm
      angleY4 = float(data[18]); // yaw4
      angleX4 = float(data[19]); // pitch4
      angleZ4 = float(data[20]); // roll4
      
      angleY5 = float(data[21]); // yaw5
      angleX5 = float(data[22]); // pitch5
      angleZ5 = float(data[23]); // roll5
      
      angleY6 = float(data[24]); // yaw6
      angleX6 = float(data[25]); // pitch6
      angleZ6 = float(data[26]); // roll6
      
      // Body rotation
      angleY9 = float(data[27]); // yaw9
    }
  }
}

void keyPressed() {
  if (key == 'r' || key == 'R') {
    resetAngles();
  }
}

void resetAngles() {
  angleX1 = angleY1 = angleZ1 = 0;
  angleX2 = angleY2 = angleZ2 = 0;
  angleX3 = angleY3 = angleZ3 = 0;
  angleX4 = angleY4 = angleZ4 = 0;
  angleX5 = angleY5 = angleZ5 = 0;
  angleX6 = angleY6 = angleZ6 = 0;
  angleY9 = 0;
}
