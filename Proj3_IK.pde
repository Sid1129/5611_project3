float dt = 1 / (60 * frameRate);

int numObs = 10;
Vec3[] centers = new Vec3[numObs];
int segLength = 30;
Camera camera;

void setup() {
  size(800, 600, P3D);
  surface.setTitle("Inverse Kinematics Project 3");
  
  for(int i = 0; i < numObs; i++){
    centers[i] = new Vec3(random(50,700),random(0,500), random(-100,100));
  }
  
  camera = new Camera();
  camera.position = new PVector(300, 198, 402);
  camera.theta = 0;
  camera.phi = -.4;
  camera.turnSpeed = 16 * PI;
  camera.moveSpeed *= 6;
  
  lights();
}

//Root
Vec3 root = new Vec3(275, 250,0);

//Segment 1
float l0 = segLength;
float a0 = 0.3; 

//Segment 2
float l1 = segLength;
float a1 = 0.3; 

//Segment 3
float l2 = segLength;
float a2 = 0.3; 

//Segment 4
float l3 = segLength;
float a3 = 0.3;

//Segment 5
float l4 = segLength;
float a4 = 0.3;

//Segment 6
float l5 = segLength;
float a5 = 0.3;

//Segment 7
float l6 = segLength;
float a6 = 0.3;

//----------------------

//Root
Vec3 rootTwo = new Vec3(325, 250,0);

//Segment 1
float r0 =segLength;
float b0 = 0.3; 

//Segment 2
float r1 = segLength;
float b1 = 0.3; 

//Segment 3
float r2 = segLength;
float b2 = 0.3; 

//Segment 4
float r3 = segLength;
float b3 = 0.3;

//Segment 5
float r4 = segLength+20;
float b4 = 0.3;

Vec3 start_l1, start_l2, start_l3, start_l4, start_l5, start_l6, endPoint;
Vec3 start_r1, start_r2, start_r3, start_r4, endPointR;
int count = 0;

void solve() {
  Vec3 goal = new Vec3(mouseX, mouseY, 0);

  Vec3 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
 
  //Update 5 joint
  //if(!listCircleIntersect(centers, radius, new Vec3(start_l4.x+l4/2,start_l4.y,0), segLength/2)){
  startToGoal = goal.minus(start_l4);
  startToEndEffector = endPoint.minus(start_l4);
  dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1, 1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal, startToEndEffector) < 0)
    a4 += angleDiff;
  else
    a4 -= angleDiff;
  fk(); //Update link positions with fk (e.g. end effector changed)
  //} 
  
  //Update 4 joint
  if(!listCircleIntersect(centers, radius, new Vec3(start_l3.x+l3/2,start_l3.y,0), segLength)){
  startToGoal = goal.minus(start_l3);
  startToEndEffector = endPoint.minus(start_l3);
  dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1, 1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal, startToEndEffector) < 0)
    a3 += angleDiff;
  else
    a3 -= angleDiff;
  fk(); //Update link positions with fk (e.g. end effector changed)
  }

  //Update wrist joint
  if(!listCircleIntersect(centers, radius, new Vec3(start_l2.x+l2/2,start_l2.y,0), segLength)){
  startToGoal = goal.minus(start_l2);
  startToEndEffector = endPoint.minus(start_l2);
  dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1, 1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal, startToEndEffector) < 0)
    a2 += angleDiff;
  else
    a2 -= angleDiff;
  if (a2 > PI/4) {
    a2 = PI/4;
  } else if (a2 < -PI/4) {
    a2 = -PI/4;
  }
  fk(); //Update link positions with fk (e.g. end effector changed)
  }


  //Update elbow joint
  if(!listCircleIntersect(centers, radius, new Vec3(start_l1.x+l1/2,start_l1.y,0), segLength)){
  startToGoal = goal.minus(start_l1);
  startToEndEffector = endPoint.minus(start_l1);
  dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1, 1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal, startToEndEffector) < 0)
    a1 += angleDiff;
  else
    a1 -= angleDiff;

  if (a1 > PI/2) {
    a1 = PI/2;
  } else if (a1 < -PI/2) {
    a1 = -PI/2;
  }
  fk(); //Update link positions with fk (e.g. end effector changed)
  }

  //Update shoulder joint
  startToGoal = goal.minus(root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint.minus(root);
  dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1, 1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal, startToEndEffector) < 0)
    a0 += angleDiff;
  else
    a0 -= angleDiff;

  if (a0 > 5*PI/4) {
    a0 = 5*PI/4;
  } else if (a0 < 3*PI/4) {
    a0 = 3*PI/4;
  }
  fk(); //Update link positions with fk (e.g. end effector changed)
}


void solveZL(){
  Vec3 goal = new Vec3(mouseX, mouseY,0);
  Vec3 startToGoalZ, startToEndEffectorZ;
  float dotProd, angleDiff;
  
      //Update 7
  startToGoalZ = goal.minus(start_l6);
  startToEndEffectorZ = start_l4.minus(start_l6);
  dotProd = dot(startToGoalZ.normalized(), startToEndEffectorZ.normalized());
  dotProd = clamp(dotProd, -1, 1);
  angleDiff = acos(dotProd);
  if (cross(startToGoalZ, startToEndEffectorZ) < 0)
    a6 += angleDiff;
  else
    a6 -= angleDiff;
  fkZ(); //Update link positions with fk (e.g. end effector changed)
  
    //Update 6
  startToGoalZ = goal.minus(start_l5);
  startToEndEffectorZ = start_l4.minus(start_l5);
  dotProd = dot(startToGoalZ.normalized(), startToEndEffectorZ.normalized());
  dotProd = clamp(dotProd, -1, 1);
  angleDiff = acos(dotProd);
  if (cross(startToGoalZ, startToEndEffectorZ) < 0)
    a5 += angleDiff;
  else
    a5 -= angleDiff;
  fkZ(); //Update link positions with fk (e.g. end effector changed)
}

void solveR() {
  Vec3 goal = new Vec3(mouseX, mouseY,0);

  Vec3 startToGoalR, startToEndEffectorR;
  float dotProd, angleDiff;
  
  //Update 5 joint
  startToGoalR = goal.minus(start_r4);
  startToEndEffectorR = endPointR.minus(start_r4);
  dotProd = dot(startToGoalR.normalized(), startToEndEffectorR.normalized());
  dotProd = clamp(dotProd, -1, 1);
  angleDiff = acos(dotProd);
  if (cross(startToGoalR, startToEndEffectorR) < 0)
    b4 += angleDiff;
  else
    b4 -= angleDiff;
  fkR(); //Update link positions with fk (e.g. end effector changed)
  
  //Update 4 joint
  startToGoalR = goal.minus(start_r3);
  startToEndEffectorR = endPointR.minus(start_r3);
  dotProd = dot(startToGoalR.normalized(), startToEndEffectorR.normalized());
  dotProd = clamp(dotProd, -1, 1);
  angleDiff = acos(dotProd);
  if (cross(startToGoalR, startToEndEffectorR) < 0)
    b3 += angleDiff;
  else
    b3 -= angleDiff;
  fkR(); //Update link positions with fk (e.g. end effector changed)

  //Update wrist joint
  startToGoalR = goal.minus(start_r2);
  startToEndEffectorR = endPointR.minus(start_r2);
  dotProd = dot(startToGoalR.normalized(), startToEndEffectorR.normalized());
  dotProd = clamp(dotProd, -1, 1);
  angleDiff = acos(dotProd);
  if (cross(startToGoalR, startToEndEffectorR) < 0)
    b2 += angleDiff;
  else
    b2 -= angleDiff;
  if (b2 > PI/4) {
    b2 = PI/4;
  } else if (b2 < -PI/4) {
    b2 = -PI/4;
  }
  fkR(); //Update link positions with fk (e.g. end effector changed)


  //Update elbow joint
  startToGoalR = goal.minus(start_r1);
  startToEndEffectorR = endPointR.minus(start_r1);
  dotProd = dot(startToGoalR.normalized(), startToEndEffectorR.normalized());
  dotProd = clamp(dotProd, -1, 1);
  angleDiff = acos(dotProd);
  if (cross(startToGoalR, startToEndEffectorR) < 0)
    b1 += angleDiff;
  else
    b1 -= angleDiff;

  if (b1 > PI/2) {
    b1 = PI/2;
  } else if (b1 < -PI/2) {
    b1 = -PI/2;
  }
  fkR(); //Update link positions with fk (e.g. end effector changed)


  //Update shoulder joint
  startToGoalR = goal.minus(rootTwo);
  if (startToGoalR.length() < .0001) return;
  startToEndEffectorR = endPointR.minus(rootTwo);
  dotProd = dot(startToGoalR.normalized(), startToEndEffectorR.normalized());
  dotProd = clamp(dotProd, -1, 1);
  angleDiff = acos(dotProd);
  if (cross(startToGoalR, startToEndEffectorR) < 0)
    b0 += angleDiff;
  else
    b0 -= angleDiff;

  if (b0 > PI/4) {
    b0 = PI/4;
  } else if (b0 < -PI/4) {
    b0 = -PI/4;
  }
  fkR(); //Update link positions with fk (e.g. end effector changed)

  //println("Angle 0:", a0, "Angle 1:", a1, "Angle 2:", a2);
}

void fk() {
  start_l1 = new Vec3(cos(a0)*l0, sin(a0)*l0,0).plus(root);
  start_l2 = new Vec3(cos(a0+a1)*l1, sin(a0+a1)*l1,0).plus(start_l1);
  start_l3 = new Vec3(cos(a0+a1+a2)*l2, sin(a0+a1+a2)*l2,0).plus(start_l2);
  start_l4 = new Vec3(cos(a0+a1+a2+a3)*l3, sin(a0+a1+a2+a3)*l3,0).plus(start_l3);
  start_l5 = new Vec3(cos(a0+a1+a2+a3+a4)*l4,sin(a0+a1+a2+a3+a4)*l4, 0).plus(start_l4);
  start_l6 = new Vec3(cos(a0+a1+a2+a3+a4+a5)*l5, sin(a0+a1+a2+a3+a4+a5)*l5,0).plus(start_l5);
  endPoint = new Vec3(cos(a0+a1+a2+a3+a4+a5+a6)*l6, sin(a0+a1+a2+a3+a4+a5+a6)*l6,0).plus(start_l6);
}

void fkR() {
  start_r1 = new Vec3(cos(b0)*r0, sin(b0)*r0,0).plus(rootTwo);
  start_r2 = new Vec3(cos(b0+b1)*r1, sin(b0+b1)*r1,0).plus(start_r1);
  start_r3 = new Vec3(cos(b0+b1+b2)*r2, sin(b0+b1+b2)*r2,0).plus(start_r2);
  start_r4 = new Vec3(cos(b0+b1+b2+b3)*r3, sin(b0+b1+b2+b3)*r3,0).plus(start_r3);
  endPointR = new Vec3(cos(b0+b1+b2+b3+b4)*r4, sin(b0+b1+b2+b3+b4)*r4,0).plus(start_r4);
}

void fkZ() {
  start_l5 = new Vec3(cos(a5)*l4,0, sin(a5)*l4).plus(start_l4);
  start_l6 = new Vec3(cos(a5+a6)*l5,0, sin(a5+a6)*l5).plus(start_l5);
}

float armW = 5;
float radius = 20;
Vec3 rootPos = new Vec3(250,175,0);
void draw() {
  camera.Update(dt);
  fk();
  solve();
  fkR();
  solveR();

  background(250, 250, 250);
  
  
  //obstacles
  fill(150, 50, 200);
  for (int i = 0; i < numObs; i++){
    Vec3 c = centers[i];
    pushMatrix();
    translate(c.x,c.y,c.z);
    sphere(radius);
    popMatrix();
  }

  //body
  fill(125, 50, 65);
  pushMatrix();
  translate(rootPos.x+50,rootPos.y+75);
  //rect(0,50,100,50);
  //ellipse(50,75,50,50);
  sphere(25);
  popMatrix();


  fill(50, 50, 65);
  pushMatrix();
  translate(root.x, root.y,0);
  rotate(a0);
  translate(l0/2,0,0);
  sphere(segLength/2);
  popMatrix();

  
  pushMatrix();
  translate(start_l1.x, start_l1.y);
  rotate(a0+a1);
  //rect(0, -armW/2, l1, armW);
  translate(l1/2,0,0);
  sphere(segLength/2);
  popMatrix();

  pushMatrix();
  translate(start_l2.x, start_l2.y);
  rotate(a0+a1+a2);
  //rect(0, -armW/2, l2, armW);
  translate(l2/2,0,0);
  sphere(segLength/2);
  popMatrix();
  
  pushMatrix();
  translate(start_l3.x, start_l3.y);
  rotate(a0+a1+a2+a3);
  //rect(0, -armW/2, l3, armW);
  translate(l3/2,0,0);
  sphere(segLength/2);
  popMatrix();
  
  pushMatrix();
  translate(start_l4.x, start_l4.y);
  rotate(a0+a1+a2+a3+a4);
  //rect(0, -armW/2, l3, armW);
  translate(l4/2,0,0);
  sphere(segLength/2);
  popMatrix();
  
  pushMatrix();
  translate(start_l5.x, start_l5.y);
  rotateX(a0+a1+a2+a3+a4+a5);
  //rect(0, -armW/2, l3, armW);
  translate(0,l5/2,0);
  sphere(segLength/2);
  popMatrix();
  
  pushMatrix();
  translate(start_l6.x, start_l6.y);
  rotateX(a0+a1+a2+a3+a4+a5+a6);
  //rect(0, -armW/2, l3, armW);
  translate(0,l6/2,0);
  sphere(segLength/2);
  popMatrix();
  
  fill(150, 150, 165);
  pushMatrix();
  translate(rootTwo.x, rootTwo.y);
  rotate(b0);
  //rect(0, -armW/2, r0, armW);
  translate(r0/2,0,0);
  sphere(segLength/2);
  popMatrix();

  pushMatrix();
  translate(start_r1.x, start_r1.y);
  rotate(b0+b1);
  //rect(0, -armW/2, r1, armW);
  translate(r1/2,0,0);
  sphere(segLength/2);
  popMatrix();

  pushMatrix();
  translate(start_r2.x, start_r2.y);
  rotate(b0+b1+b2);
  //rect(0, -armW/2, r2, armW);
  translate(r2/2,0,0);
  sphere(segLength/2);
  popMatrix();
  
  pushMatrix();
  translate(start_r3.x, start_r3.y);
  rotate(b0+b1+b2+b3);
  //rect(0, -armW/2, r3, armW);
  translate(r3/2,0,0);
  sphere(segLength/2);
  popMatrix();
  
  pushMatrix();
  translate(start_r4.x, start_r4.y);
  rotate(b0+b1+b2+b3+b4);
  //rect(0, -armW/2, r3, armW);
  translate(r4/2,0,0);
  sphere(segLength/2 + 10);
  popMatrix();
}

void keyPressed() {
  //if (key == ' ') paused = !paused;
  //if (key == 'r') initScene();
  camera.HandleKeyPressed();
}

void keyReleased() {
  camera.HandleKeyReleased();
}



boolean circleIntersect(Vec3 center, float r1, Vec3 c2, float r2){
  return center.distanceTo(c2) < (r1 + r2 + 10);
}

boolean listCircleIntersect(Vec3[] centers, float r1, Vec3 c2, float r2){
  boolean temp;
  for(int i = 0; i < centers.length; i++){
    temp = circleIntersect(centers[i], r1, c2, r2);
    if(temp){
      return true;
    }
  }
  return false;
}



//-----------------
// Vector Library
//-----------------

//Vector Library
//CSCI 5611 Vector 2 Library [Example]
// Stephen J. Guy <sjguy@umn.edu>

//public class Vec2 {
//  public float x, y;

//  public Vec2(float x, float y) {
//    this.x = x;
//    this.y = y;
//  }

//  public String toString() {
//    return "(" + x+ "," + y +")";
//  }

//  public float length() {
//    return sqrt(x*x+y*y);
//  }

//  public Vec2 plus(Vec2 rhs) {
//    return new Vec2(x+rhs.x, y+rhs.y);
//  }

//  public void add(Vec2 rhs) {
//    x += rhs.x;
//    y += rhs.y;
//  }

//  public Vec2 minus(Vec2 rhs) {
//    return new Vec2(x-rhs.x, y-rhs.y);
//  }

//  public void subtract(Vec2 rhs) {
//    x -= rhs.x;
//    y -= rhs.y;
//  }

//  public Vec2 times(float rhs) {
//    return new Vec2(x*rhs, y*rhs);
//  }

//  public void mul(float rhs) {
//    x *= rhs;
//    y *= rhs;
//  }

//  public void clampToLength(float maxL) {
//    float magnitude = sqrt(x*x + y*y);
//    if (magnitude > maxL) {
//      x *= maxL/magnitude;
//      y *= maxL/magnitude;
//    }
//  }

//  public void setToLength(float newL) {
//    float magnitude = sqrt(x*x + y*y);
//    x *= newL/magnitude;
//    y *= newL/magnitude;
//  }

//  public void normalize() {
//    float magnitude = sqrt(x*x + y*y);
//    x /= magnitude;
//    y /= magnitude;
//  }

//  public Vec2 normalized() {
//    float magnitude = sqrt(x*x + y*y);
//    return new Vec2(x/magnitude, y/magnitude);
//  }

//  public float distanceTo(Vec2 rhs) {
//    float dx = rhs.x - x;
//    float dy = rhs.y - y;
//    return sqrt(dx*dx + dy*dy);
//  }
//}

//Vec2 interpolate(Vec2 a, Vec2 b, float t) {
//  return a.plus((b.minus(a)).times(t));
//}

//float interpolate(float a, float b, float t) {
//  return a + ((b-a)*t);
//}

//float dot(Vec2 a, Vec2 b) {
//  return a.x*b.x + a.y*b.y;
//}

//float cross(Vec2 a, Vec2 b) {
//  return a.x*b.y - a.y*b.x;
//}


//Vec2 projAB(Vec2 a, Vec2 b) {
//  return b.times(a.x*b.x + a.y*b.y);
//}

float clamp(float f, float min, float max) {
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
