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
    mass = 1.0;
    radius = 6.0;
    maxspeed = 3.0;
    maxforce = 0.2;
    arrivalDistance = 200;
    wallDistance = 100;
    fleeDistance = 5;
    fleeCount = 0;
    wander = 0;
    strength = 20;
    distance = 80;
    rate = 0.04;
  }

  void applyForce( PVector force ) {
    PVector f = force.get();
    f.div(mass);
    acceleration.add( f );
  }

  void update() {
    velocity.add(acceleration);
    velocity.limit( maxspeed );
    location.add(velocity);
    acceleration.mult( 0 );
    wrap();
    
  }

  void render() {
    pushMatrix();
    translate( location.x, location.y );
    rotate( velocity.heading2D() );

    noStroke();
      fill(location.x/2, location.y/2, 255, height-location.y);
    triangle(-radius, radius * 0.8, -radius, -radius * 0.8, radius, 0);

    popMatrix();
  }

  void wrap() {
    if ((location.x + radius) < 0) location.x = width + radius;
    if ((location.x - radius) > width) location.x = 0 - radius;
    if ((location.y + radius) < 0) location.y = height + radius;
    if ((location.y - radius) > height) location.y = 0 - radius;
  }

  PVector seek( PVector target ) {
   PVector desired = PVector.sub( target, location );
   //desired = limitDesired( desired, desired.mag() );
   desired.normalize();
   desired.mult( maxspeed );
   PVector steer = PVector.sub( desired, velocity );
   steer.limit( maxforce );
   return( steer );
  }
  
  boolean isWithinFleeDistance( PVector target ) {
    float d = location.dist( target );
    if (d <= fleeDistance) {
      if (fleeCount == 0)
        fleeCount = 60;
      return true;
    }
    else
      return false;
  }
  
  void flee( PVector target ) {
    PVector desired = PVector.sub( location, target );
    desired = limitDesired( desired, desired.mag() );
    desired.normalize();
    desired.mult( maxspeed );
    PVector steer = PVector.sub( desired, velocity );
    steer.limit( maxforce );
    applyForce( steer );
  }

  PVector limitDesired( PVector desired, float d ) {
    float a = PVector.angleBetween(velocity, desired);
    float pidiv2 = PI / 3.0;
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

  void arrive( PVector target ) {
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
  
  

  void wall() {
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
  
  
   PVector arriveb(ArrayList<Vehicle> boids) {
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
  
  
  PVector separate(ArrayList<Vehicle> vehicles) {
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
  
  PVector align (ArrayList<Vehicle> boids) {
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
  
  PVector cohesion (ArrayList<Vehicle> boids) {
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
  
  
  
  PVector bound(ArrayList<Vehicle> boids){
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
  
  void boundaries() {

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
  void flock(ArrayList<Vehicle> boids) {
    PVector sep = separate(boids);     // Separation
    PVector ali = align(boids);        // Alignment
    //PVector coh = cohesion(boids);   // Cohesion
    //PVector av = avoid(boids);
    PVector ar = arriveb(boids);       // arrival behavior to slow down the vehicle
    PVector b = bound(boids);          //avoid the circle or follow it
    // Arbitrarily weight these forces
    sep.mult(1.5);
    ali.mult(1.0);
    b.mult(2.0);
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
