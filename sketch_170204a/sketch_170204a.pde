import processing.serial.*;
Serial myPort;    // The serial port
float xin=0;
float yin=0;
float zin=0;
float x, y, z;
float deg_to_rad = 0.01745329251994329576923690768489f;
float qw = 0;
float qx = 0;
float qy = 0;
float qz = 0;

float mx = 0;
float my = 0;
float mz = 0;
float ax = 0;
float ay = 0;
float az = 0;
float gx = 0;
float gy = 0;
float gz = 0;

float yaw = 0;
float pitch = 0;
float roll = 0;

float lastReadMs = 0f;

float heading = 0;
float headingDegrees = 0;

float gy_total = 0;
float gx_total = 0;
float gz_total = 0;

boolean drawMode = false;
boolean mouseDown = false;
float mouse_x = 0;
float mouse_y = 0;
float mouse_z = 0;

float[] mx_a = new float[500];
float[] my_a = new float[500];
float[] mz_a = new float[500];

void setup() {
  size(1200, 800,P3D);
  frameRate(60);
  myPort = new Serial(this, "COM12", 115200); 
  myPort.bufferUntil('\n'); 
}
void draw() {
  ortho();
  background(255);
  //drawMagCalibration();
  drawAcc(100, 100);
  drawMag(300, 100);
  drawGyro(500, 100);
  drawYPR((width / 2) - 300, (height / 2) + 100);
  drawQuat((width / 2) + 300, (height / 2) + 100);
}

void drawMagCalibration() {
  pushMatrix();
  translate(width/2, height/2);
  pushMatrix();
  rotateX(mouse_y*deg_to_rad);
  rotateY(mouse_x*deg_to_rad);
 
  // x
  strokeWeight(2);
  stroke(255,0,0);
  line(-400,0,0,400,0,0);
  // y
  stroke(0,255,0);
  line(0,-400,0,0,400,0);
  // z
  stroke(0,0,255);
  line(0,0,-400,0,0,400);
  
  float scale = 5 - mouse_z;
  sphereDetail(3,3);
      
  // draw values, 2D mode
  lights();
  strokeWeight(2);
  for (int i = 0; i < mag_counter; i++) {    
      if (!drawMode) { 
        stroke(255,100,100);
        pushMatrix();
        translate(mx_a[i] / scale, my_a[i] / scale);
        sphere(3);
        popMatrix();
        
        stroke(100,255,100);
        pushMatrix();
        translate(mx_a[i] / scale, mz_a[i] / scale);
        sphere(3);
        popMatrix();
        
        stroke(100,100,255);
        pushMatrix();
        translate(my_a[i] / scale, mz_a[i] / scale);
        sphere(3);
        popMatrix();
      }
      else {
        pushMatrix();
        stroke(100,100,100);
        translate(mx_a[i] / scale, my_a[i] / scale, mz_a[i] / scale);
        sphere(3);
        popMatrix();
      }
    }
  
  popMatrix();
  popMatrix();
}

void drawQuat(float x, float y) {
  pushMatrix();
  translate(x, y);
  
  textSize(20);
  textAlign(CENTER, BOTTOM);
  fill(100,100,100);
  text("Quaternion", 0, -220);
  
  rotateX(xin);
  rotateY(yin);
  rotateZ(zin);
  pushMatrix();
  stroke(0,0,0);
  pushMatrix();
  scale(1, 0.2, 0.4);
  box(140);
  popMatrix();
  stroke(255,0,0);
  line(0,0,0,0,-200,0);
  stroke(0,0,255);
  line(0,0,0,200,0,0);
  stroke(0,255,0);
  line(0,0,0,0,0,200);
  popMatrix();
  popMatrix();
}

void drawYPR(float x, float y) {
  pushMatrix();
  translate(x, y);
  
  textSize(20);
  textAlign(CENTER, BOTTOM);
  fill(100,100,100);
  text("Yaw, Pitch, Roll", 0, -220);
  
  rotateY(yaw * deg_to_rad);
  rotateX(pitch * deg_to_rad);
  rotateZ((roll) * deg_to_rad);

  
  pushMatrix();
  stroke(0,0,0);
  pushMatrix();
  scale(1, 0.2, 0.4);
  box(140);
  popMatrix();
  stroke(255,0,0);
  line(0,0,0,0,-200,0);
  stroke(0,0,255);
  line(0,0,0,200,0,0);
  stroke(0,255,0);
  line(0,0,0,0,0,200);
  popMatrix();
  popMatrix();
 
}

void drawAcc(float x, float y) {
  pushMatrix();
  translate(x, y);
  textSize(20);
  textAlign(CENTER, BOTTOM);
  fill(100,100,100);
  text("Accelerometer", 0, -50);
  pushMatrix();
  strokeWeight(1);
  stroke(0,0,0);
  sphere(10);
  strokeWeight(5);
  stroke(255,0,0);
  line(0,0,0,ax,ay,az);
  popMatrix();
  popMatrix();
}

void drawMag(float x, float y) {
  pushMatrix();
  translate(x, y);
  textSize(20);
  textAlign(CENTER, BOTTOM);
  fill(100,100,100);
  text("Magnetometer", 0, -50);
  rotateZ(-heading);
  pushMatrix();
  strokeWeight(1);
  stroke(0,0,0);
  sphere(10);
  strokeWeight(5);
  stroke(255,0,0);
  line(0,0,0,0,-40,0);
  popMatrix();
  popMatrix();
}

void drawGyro(float x, float y) {
  pushMatrix();
  translate(x, y);
  textSize(20);
  textAlign(CENTER, BOTTOM);
  fill(100,100,100);
  text("Gyroscope", 0, -50);
  rotateX(gx_total * deg_to_rad);
  rotateY(gy_total * deg_to_rad);
  rotateZ(gz_total * deg_to_rad);
  pushMatrix();
  strokeWeight(1);
  stroke(0,0,0);
  sphere(10);
  strokeWeight(5);
  stroke(255,0,0);
  line(0,0,0,0,-40,0);
  popMatrix();
  popMatrix();
}

void mousePressed() {
  if (mouseButton == LEFT) {
    mouseDown = true;
  }
}

void mouseReleased() {
  if (mouseButton == LEFT) {
    mouseDown = false;
  }
  else if (mouseButton == RIGHT) {
    mouse_x = 0;
    mouse_y = 0;
  }
}

void mouseDragged() {
  if (mouseDown) {
    float d_x = (mouseX - pmouseX);
    float d_y = (mouseY - pmouseY);
    mouse_x += d_x;
    mouse_y += d_y;
  }
}

void mouseWheel(MouseEvent event) {
  mouse_z += event.getCount();
  if (mouse_z < 0)
    mouse_z = 0;
}


public double[] toEulerAngles() {
      double[] ret = new double[3];

      ret[0] = Math.atan2(2 * qy * qw - 2 * qx * qz, 1 - 2 * (qy * qy) - 2
              * (qz * qz)); // atan2(2*qy*qw-2*qx*qz , 1 - 2*qy2 - 2*qz2)
      ret[1] = Math.asin(2 * qx * qy + 2 * qz * qw); // asin(2*qx*qy + 2*qz*qw) 
      ret[2] = Math.atan2(2 * qx * qw - 2 * qy * qz, 1 - 2 * (qx * qx) - 2
              * (qz * qz)); // atan2(2*qx*qw-2*qy*qz , 1 - 2*qx2 - 2*qz2)

      return ret;
  }
  
void keyPressed() {
  if (key == ' ') {
    drawMode = !drawMode;
  }
  else if (key == 'r') {
    mag_counter = 0;
  }
}

int mag_counter = 0;
int mag_interval = 0;

void serialEvent(Serial myPort) 
{ 
  String inString = myPort.readString();
  if (inString.charAt(0) == '&') 
  {
    if (inString.charAt(inString.length() - 3) == '|')
    {
      inString = inString.substring(0, inString.length() - 3);
    }
    
    float[] vals = float(split(inString,","));
    
    qw = vals[1];
    qx = vals[2];
    qy = vals[3];
    qz = vals[4];
    
    gx = vals[5];
    gy = vals[6];
    gz = vals[7];
    
    yaw = vals[8];
    pitch= vals[9];
    roll = vals[10];
    
    
    /*ax = vals[5];
    ay = vals[6];
    az = vals[7];
    
    float d = sqrt(ax * ax + ay * ay + az * az);
    float multiplier = 40;
    ax /= d / multiplier;
    ay /= d / multiplier;
    az /= d / multiplier;
    
    gx = vals[8];
    gy = vals[9];
    gz = vals[10];
    
    mx = vals[11];
    my = vals[12];
    mz = vals[13];
    
    yaw = vals[14];
    pitch= vals[15];
    roll = vals[16];*/
    
    heading = atan2(my, mx);
    
    if(heading > 2*PI)
      heading -= 2*PI;
    
    headingDegrees = heading * 180/PI; // The heading in Degrees unit
    
    
    double[] euler = toEulerAngles();
    xin = (float) euler[0];
    yin = (float) euler[1];
    zin = (float) euler[2];
    
    float timeElapsed = millis() - lastReadMs;
    lastReadMs = millis();
    //println("Time elapsed: ", timeElapsed);
    
    gx_total += gx * (timeElapsed / 1000f);
    gy_total += gy * (timeElapsed / 1000f);
    gz_total += gz * (timeElapsed / 1000f);
    
    if (mag_counter < 500) {
      if (mag_interval >= 1) {
        mag_interval = 0;
        mx_a[mag_counter] = mx;
        my_a[mag_counter] = my;
        mz_a[mag_counter] = mz;
        mag_counter++;
        //println("Mag value read:", mx, my, mz, mag_counter);
      }
      else {
        mag_interval++;
      }
    }
    
    println("yaw:", nf(yaw, 3 ,6), "pitch:", nf(pitch, 3, 6), "roll:", nf(roll, 3, 6));
    //println("MX: ", mx, "MY: ", my, "MZ: ", mz, "heading: ", headingDegrees);
  }
  else {
    print(inString);
  }
}