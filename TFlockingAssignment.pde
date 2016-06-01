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


void setup() {
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

void draw() {
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

void mouseDragged() {
  if(mouseButton ==RIGHT){
    target = new PVector(mouseX, mouseY);
  }
  else if (mouseButton == LEFT){

  circleLocation = new PVector(mouseX, mouseY);
  }
  
}

void mousePressed() {
if (mouseButton == RIGHT) {
   circleLocation2 = new PVector(mouseX, mouseY);
 }
else if(mouseButton == LEFT){
  // target = new PVector(mouseX, mouseY);
}
}