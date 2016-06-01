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
    mass = 1.0;
    radius = 6.0;
    maxSpeed = 2;
    maxForce = 0.05;
    arrivalDistance = 100;
    
   
    wanderTheta = 0;
    R = 87;
    G = 235;
    B = 102;
    startTimer = false;
    
  }//end of constructor

  void applyForce( PVector force ) {
    PVector f = force.get();
    f.div(mass);
    acceleration.add( f );
  }
  
  void update() {
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

  void render() {
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

  void wrap() {
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
 void wander() {
   
    float strength = 25;                             // strength for our wander circle
    float rate = 80;                                 // rate for our wander circle
    float change = 0.3;
    wanderTheta += random(-change,change);           // Randomly change wander theta
    
    // Now we have to calculate the new location to steer towards on the wander circle
    PVector circleloc = velocity.get();            
    circleloc.normalize();                 
    circleloc.mult(rate);          
    circleloc.add(location);               
    
    //Use noise to offset wandertheta
    float t = frameCount / 60.0;
    float u = circleloc.x /400.0 + t;
    float v = circleloc.y / 400.0 + t;
    float n = (noise(u,v) - 0.5) * 0.02;
    wanderTheta += n;
    
    PVector circleOffSet = new PVector(strength*cos(wanderTheta),strength*sin(wanderTheta));
    PVector target = PVector.add(circleloc,circleOffSet);
    seek(target);
 }
    
  
  
 void seek(PVector target){
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
 void movement(PVector target){
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
 void boundaries() {

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