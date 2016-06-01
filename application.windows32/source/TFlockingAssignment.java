import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class TFlockingAssignment extends PApplet {

// Experiments with Autonomous Characters
// Flocking Demo (Reynolds Boids)
// Shiffman Chapter 6.

ArrayList<Vehicle> vehicles;
ArrayList<Vehicle2> vehicles2;
Vehicle v;
Vehicle2 v2;
boolean debug = true;
String checker = " ";

PVector circleLocation;
PVector circleBoundary;
float circleRadius;


PVector circleLocation2;
PVector circleBoundary2;
float circleRadius2;

PVector target;


public void setup() {
  size( 800 , 800 );
  vehicles = new ArrayList<Vehicle>();
  for (int i = 0; i < 50; i++) {
    v = new Vehicle( random(100, width-100), random(100,height-100) );
    vehicles.add( v );
  }
  
  vehicles2 = new ArrayList<Vehicle2>();
  for (int i = 0; i < 10; i++) {
    v2 = new Vehicle2( random(100, width-100), random(100,height-100) );
    vehicles2.add( v2 );
  }
  
  circleLocation = new PVector(width/2-100,height/2+100);
  circleRadius = 200;
  circleBoundary = new PVector(20,20);
  
  circleLocation2 = new PVector(width/4+200,height/4);
  circleRadius2 = 100;
  circleBoundary2 = new PVector(20,20);
  
  target = new PVector( width/2+335, height/2+145);
  
}

public void draw() {
  background( 0,0,25 );
  
  PVector m = new PVector( mouseX, mouseY );
  fill(240,195,82);
  ellipse(target.x, target.y, 20,20); //draw pacman
  ellipseMode(CENTER);
  
  for (Vehicle v : vehicles) {
    
    v.flock( vehicles );
   // v.boundaries( );
    //v.wall();
    //v.seek(circleLocation);
    //v.flee(circleBoundary);
    v.update();
    v.render();
  }
  
    for (Vehicle2 v2 : vehicles2) {
    
      
    //v2.wander( );
    //v.wall();
    v2.boundaries( );
    v2.movement(target);
    v2.update();
    v2.render();
 }
//
  if (debug) {      //check if vehicles are in circle
   stroke(175);
   noFill();
   ellipse(circleLocation.x,circleLocation.y, circleRadius*2,circleRadius*2);
   ellipse(circleLocation2.x,circleLocation2.y, circleRadius2*2,circleRadius2*2);
  }
  
  fill(255);
  textSize(10);
  text("Mouse Left Drag = move large circle", width/2+125, height/2+100);
  text("Mouse Right Click = move small circle", width/2+125, height/2+125);  
  text("Mouse Right Drag = move yellow circle", width/2+125, height/2+150);
  text("If yellow circle is in the small circle,", width/2+125, height/2+175);
  text("the other cicles seek the yellow circle", width/2+125, height/2+200); 
}

public void mouseDragged() {
  if(mouseButton ==RIGHT){
    target = new PVector(mouseX, mouseY);
  }
  else if (mouseButton == LEFT){

  circleLocation = new PVector(mouseX, mouseY);
  }
  
}

public void mousePressed() {
if (mouseButton == RIGHT) {
   circleLocation2 = new PVector(mouseX, mouseY);
 }
else if(mouseButton == LEFT){
  // target = new PVector(mouseX, mouseY);
}
}
// An "idealized Vehicle" - for experimenting with Autonomous Characters

class Vehicle {
  PVector location;
  PVector velocity;
  PVector acceleration;
  float mass;
  float radius;
  float maxspeed;
  float maxforce;
  float arrivalDistance;
  float wallDistance;
  float fleeDistance;
  int fleeCount;
  float strength;
  float wander;
  float distance;
  float rate;

  Vehicle( float _x, float _y ) {
    location = new PVector( _x, _y );
    velocity = new PVector( random( -3, 3 ), random( -3, 3 ) );
    //velocity = new PVector( 0, 0 );
    acceleration = new PVector( 0, 0 );
    mass = 1.0f;
    radius = 6.0f;
    maxspeed = 3.0f;
    maxforce = 0.2f;
    arrivalDistance = 200;
    wallDistance = 100;
    fleeDistance = 5;
    fleeCount = 0;
    wander = 0;
    strength = 20;
    distance = 80;
    rate = 0.04f;
  }

  public void applyForce( PVector force ) {
    PVector f = force.get();
    f.div(mass);
    acceleration.add( f );
  }

  public void update() {
    velocity.add(acceleration);
    velocity.limit( maxspeed );
    location.add(velocity);
    acceleration.mult( 0 );
    wrap();
    
  }

  public void render() {
    pushMatrix();
    translate( location.x, location.y );
    rotate( velocity.heading2D() );

    noStroke();
      fill(location.x/2, location.y/2, 255, height-location.y);
    triangle(-radius, radius * 0.8f, -radius, -radius * 0.8f, radius, 0);

    popMatrix();
  }

  public void wrap() {
    if ((location.x + radius) < 0) location.x = width + radius;
    if ((location.x - radius) > width) location.x = 0 - radius;
    if ((location.y + radius) < 0) location.y = height + radius;
    if ((location.y - radius) > height) location.y = 0 - radius;
  }

  public PVector seek( PVector target ) {
   PVector desired = PVector.sub( target, location );
   //desired = limitDesired( desired, desired.mag() );
   desired.normalize();
   desired.mult( maxspeed );
   PVector steer = PVector.sub( desired, velocity );
   steer.limit( maxforce );
   return( steer );
  }
  
  public boolean isWithinFleeDistance( PVector target ) {
    float d = location.dist( target );
    if (d <= fleeDistance) {
      if (fleeCount == 0)
        fleeCount = 60;
      return true;
    }
    else
      return false;
  }
  
  public void flee( PVector target ) {
    PVector desired = PVector.sub( location, target );
    desired = limitDesired( desired, desired.mag() );
    desired.normalize();
    desired.mult( maxspeed );
    PVector steer = PVector.sub( desired, velocity );
    steer.limit( maxforce );
    applyForce( steer );
  }

  public PVector limitDesired( PVector desired, float d ) {
    float a = PVector.angleBetween(velocity, desired);
    float pidiv2 = PI / 3.0f;
    if (abs( a ) > pidiv2) {
      float desiredHeading;
      if (a > 0) {
        desiredHeading = velocity.heading2D() + pidiv2;
      } else {
        desiredHeading = velocity.heading2D() - pidiv2;
      }
      desired = PVector.fromAngle(desiredHeading);
      desired.setMag( d );
      //line( location.x, location.y, location.x + desired.x, location.y + desired.y );
    }
    return desired;
  }

  public void arrive( PVector target ) {
    PVector desired = PVector.sub( target, location );
    float d = desired.mag();

    desired = limitDesired( desired, d );
    desired.normalize();
    if (d < arrivalDistance) {
      float m = map( d, 0, arrivalDistance, 0, maxspeed );
      desired.mult( m );
    } else {
      desired.mult( maxspeed );
    }
    PVector steer = PVector.sub( desired, velocity );
    steer.limit( maxforce );
    applyForce( steer );
  }
  
  

  public void wall() {
    PVector desired = null;

    if (location.x < wallDistance) {
      desired = new PVector(maxspeed, velocity.y);
    } 
    else if (location.x > width -wallDistance) {
      desired = new PVector(-maxspeed, velocity.y);
    } 

    if (location.y < wallDistance) {
      desired = new PVector(velocity.x, maxspeed);
    } 
    else if (location.y > height-wallDistance) {
      desired = new PVector(velocity.x, -maxspeed);
    } 

    if (desired != null) {
      desired.normalize();
      desired.mult(maxspeed);
      PVector steer = PVector.sub(desired, velocity);
      steer.limit(maxforce);
      applyForce(steer);
    }
  }
  
  
   public PVector arriveb(ArrayList<Vehicle> boids) {
     PVector steer = new PVector(0,0);
     PVector desired = new PVector(0,0);
     for (Vehicle other : vehicles){
        desired = PVector.sub( location, other.location );
        float d = desired.mag();
    
        desired = limitDesired( desired, d );
        desired.normalize();
        if (d < arrivalDistance) {
          float m = map( d, 0, arrivalDistance, 0, maxspeed );
          desired.mult( m );
        } else {
          desired.mult( maxspeed );
        }
     }
     steer = PVector.sub( desired, velocity );
      steer.limit( maxforce );
      applyForce( steer );
     
     return steer;
  }
  
  
  public PVector separate(ArrayList<Vehicle> vehicles) {
    PVector steer = new PVector( 0, 0 );
    float desiredseparation = radius*4;
    PVector sum = new PVector();
    int count = 0;
    // For every boid in the system, check if it's too close
    for (Vehicle other : vehicles) {
      float d = PVector.dist(location, other.location);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredseparation)) {
        // Calculate vector pointing away from neighbor
        PVector diff = PVector.sub(location, other.location);
        diff.normalize();
        diff.div(d);        // Weight by distance
        sum.add(diff);
        count++;            // Keep track of how many
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      // Our desired vector is moving away maximum speed
      sum.setMag(maxspeed);
      // Implement Reynolds: Steering = Desired - Velocity
      steer = PVector.sub(sum, velocity);
      steer.limit(maxforce);
      //applyForce(steer);
    }
    return steer;
  }
  
  public PVector align (ArrayList<Vehicle> boids) {
    float neighbordist = 50;
    PVector sum = new PVector(0,0);
    int count = 0;
    for (Vehicle other : boids) {
      float d = PVector.dist(location,other.location);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.velocity);
        count++;
      }
    }
    if (count > 0) {
      sum.div((float)count);
      sum.normalize();
      sum.mult(maxspeed);
      PVector steer = PVector.sub(sum,velocity);
      steer.limit(maxforce);
      return steer;
    } else {
      return new PVector(0,0);
    }
  }
  
  public PVector cohesion (ArrayList<Vehicle> boids) {
    float neighbordist = 50;
    PVector sum = new PVector(0,0);   // Start with empty vector to accumulate all locations
    int count = 0;
    for (Vehicle other : boids) {
      float d = PVector.dist(location,other.location);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.location); // Add location
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      return seek(sum);  // Steer towards the location
    } else {
      return new PVector(0,0);
    }
  }
  
  
  
  public PVector bound(ArrayList<Vehicle> boids){
       PVector desired = null;
     PVector steer = new PVector (0, 0);
    // Predict location 25 (arbitrary choice) frames ahead
    PVector predict = velocity.get();
    predict.mult(60);
    PVector futureLocation = PVector.add(location, predict);
    float distance = PVector.dist(futureLocation,circleLocation);
    
    if (distance > circleRadius) {
      PVector toCenter = PVector.sub(circleLocation,location);
      toCenter.normalize();
      toCenter.mult(velocity.mag());
      desired = PVector.add(velocity,toCenter);
      desired.normalize();
      desired.mult(maxspeed);
    }
   
    if (desired != null) {
      steer = PVector.sub(desired, velocity);
      steer.limit(maxforce);
      //applyForce(steer);
      //return steer;
    }
      return steer;
  }
  
  public void boundaries() {

    PVector desired = null;
    
    // Predict location 25 (arbitrary choice) frames ahead
    PVector predict = velocity.get();
    predict.mult(20);
    PVector futureLocation = PVector.add(location, predict);
    float distance = PVector.dist(futureLocation,circleLocation);
    
    if (distance > circleRadius) {
      PVector toCenter = PVector.sub(circleLocation,location);
      toCenter.normalize();
      toCenter.mult(velocity.mag());
      desired = PVector.add(velocity,toCenter);
      desired.normalize();
      desired.mult(maxspeed);
    }

    if (desired != null) {
      PVector steer = PVector.sub(desired, velocity);
      steer.limit(maxforce);
      applyForce(steer);
    }
    
    fill(255,0,0);
    ellipse(futureLocation.x,futureLocation.y,4,4);
    
  }
  
  // We accumulate a new acceleration each time based on three rules
  public void flock(ArrayList<Vehicle> boids) {
    PVector sep = separate(boids);     // Separation
    PVector ali = align(boids);        // Alignment
    //PVector coh = cohesion(boids);   // Cohesion
    //PVector av = avoid(boids);
    PVector ar = arriveb(boids);       // arrival behavior to slow down the vehicle
    PVector b = bound(boids);          //avoid the circle or follow it
    // Arbitrarily weight these forces
    sep.mult(1.5f);
    ali.mult(1.0f);
    b.mult(2.0f);
    //ar.mult(-0.1);
    //coh.mult(1.0);
    //av.mult(1.0);
    // Add the force vectors to acceleration
    applyForce(sep);
    applyForce(ali);
    applyForce(b);
    applyForce(ar);
    //applyForce(coh);
   //applyForce(av);
  }
  
}
// An "idealized Vehicle" - for experimenting with Autonomous Characters

class Vehicle2 {
  PVector location;
  PVector velocity;
  PVector acceleration;
  float mass;
  float radius;
  float maxSpeed; 
  float maxForce;
  float arrivalDistance;
  
  //wander variables
  float wanderTheta;
 
 
  
  //timer variables
  boolean startTimer;
  float time;
  
  //ref:http://www.openprocessing.org/sketch/45098
  //variables to change the color ranges, RGB based
  float R=125;
  float centerR=125;
  float a = PI/2;
  float a1 = PI/4;
  float a2 = PI;
  float pathR=125;
  float pathG=50;
  float G=50;
  float centerG=125;
  float pathB=125;
  float B=125;
  float centerB=125;
  //end color variables
  
  
  Vehicle2( float _x, float _y ) {
    location = new PVector( _x, _y );
    velocity = new PVector(random(-3, 3), random(-3,3));
    acceleration = new PVector( 0, 0 );
    mass = 1.0f;
    radius = 6.0f;
    maxSpeed = 2;
    maxForce = 0.05f;
    arrivalDistance = 100;
    
   
    wanderTheta = 0;
    R = 87;
    G = 235;
    B = 102;
    startTimer = false;
    
  }//end of constructor

  public void applyForce( PVector force ) {
    PVector f = force.get();
    f.div(mass);
    acceleration.add( f );
  }
  
  public void update() {
    velocity.add(acceleration);
    location.add(velocity);
    acceleration.mult( 0 );
    wrap();
    
    //color changer
    pathR=centerR+R*sin(a);
    a=+ random(-3,location.y); 
    pathG=centerG+G*sin(a1);
    a1=a1+random(-3,location.y);  
    pathB=centerB+B*sin(a2);
    a2=a2+random(-3,location.y);
    //end color changer
    
    
  }

  public void render() {
    float theta = velocity.heading2D() + radians(90);    
    
    pushMatrix();
    translate( location.x, location.y );
    //rotate( velocity.heading2D() );
    rotate(theta);
   
    noStroke();
    strokeWeight(2);
    fill(pathR, pathG, 255);
    //beginShape(TRIANGLES);
    //vertex(0, -radius*2);
    //vertex(-radius, radius*2);
    //vertex(radius, radius*2);
    
    ellipse(-radius,radius, radius*2, radius*2);
    
    rectMode(RADIUS);
     
    
    popMatrix();
    
    //for debugging
    //println(arrivalDistance);
    //println(startTimer);
    //println(time);
  }//end of render

  public void wrap() {
    if ((location.x + radius) < 0) location.x = width + radius;
    if ((location.x - radius) > width) location.x = 0 - radius;
    if ((location.y + radius) < 0) location.y = height + radius;
    if ((location.y - radius) > height) location.y = 0 - radius;
  }
  
  //notes: seek vs pursuit:
  //"skate to where the puck is going to be" not where the puck is
  //so where the target is going to be
  //opposite of this is evasion, opposite of seek is flee
 
 //using shipman's wandering 
 public void wander() {
   
    float strength = 25;                             // strength for our wander circle
    float rate = 80;                                 // rate for our wander circle
    float change = 0.3f;
    wanderTheta += random(-change,change);           // Randomly change wander theta
    
    // Now we have to calculate the new location to steer towards on the wander circle
    PVector circleloc = velocity.get();            
    circleloc.normalize();                 
    circleloc.mult(rate);          
    circleloc.add(location);               
    
    //Use noise to offset wandertheta
    float t = frameCount / 60.0f;
    float u = circleloc.x /400.0f + t;
    float v = circleloc.y / 400.0f + t;
    float n = (noise(u,v) - 0.5f) * 0.02f;
    wanderTheta += n;
    
    PVector circleOffSet = new PVector(strength*cos(wanderTheta),strength*sin(wanderTheta));
    PVector target = PVector.add(circleloc,circleOffSet);
    seek(target);
 }
    
  
  
 public void seek(PVector target){
    PVector desired = PVector.sub(target, location);   //desired velocity
    desired.normalize();
    desired.mult(maxSpeed *2);
    
    //steering
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxForce);
    applyForce(steer);
  }//end of seek
  
  
 /*movement is a modification of the arrival function
 Here, we check if the circle is chasing after the triangles by setting the arrivalDistance 
 to the range when the triangles have to flee
 */
 public void movement(PVector target){
    PVector desired = PVector.sub(target, location);   //desired velocity
    desired.mult(-1);                                 //multiply the Vector by -1 helps the target only be seeked in the circle
    
    float d = desired.mag(); // this is the distance
    desired.normalize();
    
    //actions
    wander();
    boundaries();
     if (d < arrivalDistance){
    seek(target);
   }
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxForce);
    applyForce(steer);
  }//end of movement
  

//example of shiffman's boundaries 
//Here the agent will be within a circle
 public void boundaries() {

    PVector desired = null;
    
    // Predict location 25 (arbitrary choice) frames ahead
    //found out that this will allow the agent to follow the circle contiously
    PVector predict = velocity.get();
    predict.mult(25);
    PVector futureLocation = PVector.add(location, predict);
    float distance = PVector.dist(futureLocation,circleLocation2);
    
    if (distance > circleRadius2) {
      PVector toCenter = PVector.sub(circleLocation2,location);
      toCenter.normalize();
      toCenter.mult(velocity.mag());
      desired = PVector.add(velocity,toCenter);
      desired.normalize();
      desired.mult(maxSpeed);
    }

    if (desired != null) {
      PVector steer = PVector.sub(desired, velocity);
      steer.limit(maxForce);
      applyForce(steer);
    }
    
    
  }
  
 }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "--full-screen", "--bgcolor=#666666", "--stop-color=#cccccc", "TFlockingAssignment" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
