
/**
 *  This complements the SVG image reader.
 *  Recieves coordinate data via serial.
 *  Controls motors for x and y axes as well as raising and lowering a pen.
 *  The exact details of the motor control will have to be changed
 *  to match your setup.
 *  
 *  Copyright 2012 Eric Heisler
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3 as published by
 *  the Free Software Foundation.
 *  
 *  Redistribute by Je7sen 2013
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  The SVG vector graphics file type is specified by and belongs to W3C
 */
#include <Servo.h>
#include <AFMotor.h>
#include <AccelStepper.h>

//////////////////////////////////////////////
// Set these variables to match your setup
//////////////////////////////////////////////
// step parameters
const float stepSize[2] = {0.07,0.3}; // mm per step [x,y]

// this is for four-wire Unipolar steppers
AF_Stepper motor_y(200, 1);
AF_Stepper motor_x(200, 2);

// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
void forwardstep1() {  
  motor_x.onestep(FORWARD, SINGLE);
}
void backwardstep1() {  
  motor_x.onestep(BACKWARD, SINGLE);
}
// wrappers for the second motor!
void forwardstep2() {  
  motor_y.onestep(FORWARD, SINGLE);
}
void backwardstep2() {  
  motor_y.onestep(BACKWARD, SINGLE);
}

// Motor shield has two motor ports, now we'll wrap them in an AccelStepper object
AccelStepper stepper_x(forwardstep1, backwardstep1);
AccelStepper stepper_y(forwardstep2, backwardstep2);


//z axis servo motor
const int PEN_SERVO_PIN = 9;

int penAngleUp = 120;
int penAngleDown = 90;

Servo penServo;

// the current position
float posmm[2];
int poss[2];

// used by the drawing function
int xSteps, ySteps;
int xdir, ydir;
float slope;
int dx, dy;

// used for serial communication
char inputchars[10];
int charcount;
float newx, newy;
int sign;
boolean started;
boolean zdown;
boolean xfinish;

void setup(){
    
    stepper_x.setMaxSpeed(800.0);
    stepper_x.setAcceleration(100.0);
    stepper_x.moveTo(5950);
    
    stepper_y.setMaxSpeed(800.0);
    stepper_y.setAcceleration(100.0);
    stepper_y.moveTo(1600);
  

  //pen servo
  penServo.attach(PEN_SERVO_PIN);
  penServo.write(penAngleUp);
  delay(500);

  // initialize the numbers
  posmm[0] = 0.0;
  posmm[1] = 0.0;
  poss[0] = 0;
  poss[1] = 0;
 
  // set up the serial stuff
  Serial.begin(9600);
  started = false;
  zdown = false;
  
  posmm[0] = 0.0;
  posmm[1] = 0.0;
  poss[0] = 0;
  poss[1] = 0;
  
}


void loop() {
  
  Serial.println(Serial.available());
  
  // wait for data to come
  while(Serial.available() < 1)
  {
    delay(10);
  }
  // start if the char 'S' is sent, finish if 'T' is sent
  if(Serial.peek() == 'S')
  {
    // drawing started
    started = true;
    zdown = false;
    Serial.println("S");
    Serial.read();
    delay(10);
   }
   
  else if(Serial.peek() == 'T')
  {
    // drawing finished
    started = false;
    Serial.println("T");
    Serial.read();
    delay(10);
    raisePen();
    drawLine(0.0, 0.0);
    delay(10);
    //disable motor to save power
    motor_x.release();
    motor_y.release();
    
    posmm[0] = 0.0;
    posmm[1] = 0.0;
    poss[0] = 0;
    poss[1] = 0;
    
  }
  
  else if(Serial.peek() == 'U')
  {
    stepper_x.setSpeed(250); 
    stepper_x.move(142);  
    stepper_x.runToPosition();
    //disable motor to save power
    motor_x.release();
    motor_y.release();
  }
  
   else if(Serial.peek() == 'X')
  {
    stepper_x.setSpeed(-250); 
    stepper_x.move(-142);  
    stepper_x.runToPosition();
    //disable motor to save power
    motor_x.release();
    motor_y.release();
  } 
  
  else if(Serial.peek() == 'L')
  {
    stepper_y.setSpeed(-150); 
    stepper_y.move(-33);  
    stepper_y.runToPosition();
    //disable motor to save power
    motor_x.release();
    motor_y.release();
  } 
  
  else if(Serial.peek() == 'R')
  {
    stepper_y.setSpeed(150); 
    stepper_y.move(33);  
    stepper_y.runToPosition();
    //disable motor to save power
    motor_x.release();
    motor_y.release();
  } 
  
  else if(Serial.peek() == 'E')
  {
    
    //disable motor to save power
    motor_x.release();
    motor_y.release();
    
    posmm[0] = 0.0;
    posmm[1] = 0.0;
    poss[0] = 0;
    poss[1] = 0;
  }
  
  else if(Serial.peek() == 'A')
  {
    // raise pen
    Serial.println("A");
    Serial.read();
    if(zdown)
    {
      raisePen();
      zdown = false;
    }
    
    Serial.write(7);
    delay(10);
  }
  
  else if(Serial.peek() == 'Z')
  {
    // lower pen
    Serial.println("Z");
    Serial.read();
    if(!zdown)
    {
      lowerPen();
      zdown = true;
    }
    
    Serial.write(7);
    delay(10);
  }
  
  else 
  {
    // if there is some serial data, read it, parse it, use it
    boolean complete = false;
    char tmpchar;
    if (Serial.available() > 0) 
    {
      charcount = 0;
      complete = false;
      newx = 0;
      sign = 1;
      while(!complete){
        
        while(Serial.available() < 1){
          delay(1);
        }
        tmpchar = Serial.read();
        if(tmpchar == '.'){
          complete = true;
          xfinish = true;
          Serial.println(".x");
          delay(10);
        }
        else if(tmpchar == '-'){
          sign = -1;
          Serial.println("-x");
          delay(10);
          
        }else{
          newx = (newx*10.0) + tmpchar-'0';
          Serial.println(newx);
          delay(10);
        }
        charcount++;
        while(Serial.available() > 0){
        Serial.read(); // clear the port
      }
      }
      newx = (newx*sign)/10000.0;
      Serial.println(newx);
      while(Serial.available() > 0){
        Serial.read(); // clear the port
      }
      Serial.write(charcount); // write a verification byte
      delay(10);
    }
    // wait for the y data
    //while(Serial.available() < 1){
     //delay(10);
    //}
    if (xfinish ) {
      charcount = 0;
      complete = false;
      newy = 0;
      sign = 1;
      while(Serial.available() > 0){
        Serial.read(); // clear the port
      }
      while(!complete){
        
        while(Serial.available() < 1){
          delay(10);
        }
        tmpchar = Serial.read();
        if(tmpchar == '.'){
          complete = true;
          xfinish = false;
          Serial.println(".y");
          delay(10);

        }
        else if(tmpchar == '-'){
          sign = -1;
          Serial.println("-y");
          delay(10);
          
        }else{
          newy = (newy*10.0) + tmpchar-'0';
          Serial.println(newy);
          delay(10);       
          
        }
        charcount++;
        while(Serial.available() > 0){
        Serial.read(); // clear the port
      }
      }
      newy = (newy*sign)/10000.0;
      Serial.println(newy);
      while(Serial.available() > 0){
        Serial.read(); // clear the port
      }
      Serial.write(charcount); // send verification byte
      delay(10);
    }
    // now we have newx and newy. 
    drawLine(newx, newy);
    delay(10);
    //release motor to save power
    motor_x.release();
    motor_y.release();
    Serial.write('D');
    delay(10);
    
  }
  while(Serial.available()>0)
  {
    Serial.read();
  }
  
}


void raisePen(){

  if (isPenDown()) {
    penServo.write(penAngleUp);
    delay(10);
  }
}

void lowerPen(){

  if (!isPenDown()) {
    penServo.write(penAngleDown);
    delay(10);
}}

boolean isPenDown() {
  return penServo.read() == penAngleDown;
}


void oneStep(int m, int dir, int xspeed, int yspeed){
  // make one step with motor number m in direction dir
  if(dir > 0){
    if(m==0)
    {
    poss[m]++;
    posmm[m] += stepSize[m];
    stepper_x.setSpeed(xspeed); 
    stepper_x.move(1);  
    stepper_x.runToPosition();
    }
    
    else
    {
      poss[m]++;
      posmm[m] += stepSize[m];
      stepper_y.setSpeed(yspeed);
      stepper_y.move(1);
      stepper_y.runToPosition();
    }
    
  }
  else{

    if(m==0)
    {
    poss[m]--;
    posmm[m] -= stepSize[m];
    stepper_x.setSpeed(-xspeed);
    stepper_x.move(-1);
    stepper_x.runToPosition();
    }
    
    else
    {
      poss[m]--;
      posmm[m] -= stepSize[m];
      stepper_y.setSpeed(-yspeed);
      stepper_y.move(-1);
      stepper_y.runToPosition();
    }
  }

}

/*
* moves the pen in a straight line from the current position
* to the point (x2, y2)
*/
void drawLine(float x2, float y2){

  // determine the direction and number of steps
  xdir = 1;
  if(x2-posmm[0] < 0 ) xdir = -1;
  xSteps = int((x2-posmm[0])/stepSize[0] + 0.5*xdir);
  ydir = 1;
  if(y2-posmm[1] < 0) ydir = -1;
  ySteps = int((y2-posmm[1])/stepSize[1] + 0.5*ydir);
  if(xSteps*xdir > 0){
    slope = ySteps*1.0/(1.0*xSteps)*ydir*xdir;
  }else{
    slope = 9999;
  }
  dx = 0;
  dy = 0;

  if(xSteps*xdir > ySteps*ydir){
    while(dx < xSteps*xdir){
     
      // move one x step at a time
      dx++;
      oneStep(0, xdir ,450,200);
      // if needed, move y one step
      
        if(ySteps*ydir > 0 && slope*dx > dy){
        dy++;
        oneStep(1, ydir, 450 , 200);
      }
      
      
    
    }
    
  }
  else{
    while(dy < ySteps*ydir){
      
      // move one y step at a time
      dy++;
      oneStep(1, ydir, 450,200);
      // if needed, move y one step
      
      if(xSteps*xdir > 0 && dy > slope*(dx)){
        dx++;
        oneStep(0, xdir,450,200);
      }
    }
    
  }
  // at this point we have drawn the line
}


