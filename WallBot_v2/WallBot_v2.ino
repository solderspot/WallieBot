// Copyright (c) 2014, Solder Spot
// All rights reserved. 
// See license.txt 

// include needed libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <SS_Servorator.h>
#include <SS_ServoTrim.h>
#include <Servo.h>

// Bot states
enum State
{
  IDLE,
  MOVING,
  STOPPING,
  TURNING,
  NUM_STATES
};

#define LEFT  1
#define RIGHT 0

// for debug output
char *stateName[NUM_STATES] = 
{
  "idle",
  "moving",
  "stopping",
  "turning"
};

// get motor shield set up
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *lm = AFMS.getMotor(4);
Adafruit_DCMotor *rm = AFMS.getMotor(3);

// Servo control using Servo, SS_Servorator and SS_ServoTrim libraries
#define NUM_SERVOS 1

Servo servo[NUM_SERVOS];
SS_Servorator sr(NUM_SERVOS);
SS_ServoTrim trim(NUM_SERVOS);

int trigPin = A0;
int echoPin = A1;

State state = IDLE;
int lastDistance = 0;
int lastState = -1;
int stopDistance = 600;    // 100ths of inches
int startDistance = 1000;  // 100ths of inches
int turnSpeed = 20;        // 0..255
int moveSpeed = 70;        // 0..s55

SS_Angle min_angle = SS_DEGREES(45);
SS_Angle max_angle = SS_DEGREES(155);


// forward references
void outputDebug( int distance );
void stopMoving( void );
void turn( void );
void moveForward( void );

void setup() 
{
  // set up Serial library at 9600 bps
  Serial.begin(9600);
  Serial.println("Wall Bot V2!");
  
  // set up motors 
  AFMS.begin();
  lm->run(RELEASE);
  rm->run(RELEASE);
 
   // assign PWM pins to servos
  servo[0].attach(3);
  
  // register servo handler
  sr.setServoHandler( update_servo, NULL);

  // set initial position and speed of servo
  sr.setServoTargetAngle( 0, SS_DEGREES(90)); 
  sr.setServoMaxVelocity( 0, SS_NORMAL_RATE*2 );

  // set up trims so servo ranges are correct
  trim.setServoPulseRange( 0, 500, 2500);

  // wait for servo to get into starting position
  while ( sr.getServoAngle(0) != SS_DEGREES(90))
  {
      sr.service();
  }

  //delay a second so we can verify the servo is correctly centered
  // if not use the trim to adjust center point. 
  delay(1000);
  
  // setup sonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() 
{
  // read distance from sensor
  int distance = readDistance();

  // output debug info
  outputDebug( distance/100 );

  // handle servo logic
  control_servos();
  
  // some triggers
  int canMove = distance >= startDistance;
  int mustStop = distance <= stopDistance;
  
  // handle each state
  switch(state)
  {
    case IDLE :
    case TURNING :
    {
      if( canMove )
      {
        moveForward();
      }
      else if (state != TURNING)
      {
        turn();
      }
      break;
    }
    
    case MOVING :
    {
      if ( mustStop )
      {
        stopMoving();
      }
      break;
    }
    
    case STOPPING:
    {
      delay(500);
      turn();
      break; 
    }
   }
   // small delay or sensor
   // will get over triggered
   delay(100);
}

// returns 100ths of inches
int readDistance()
{
  // reset trig pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // pulse trig pin
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);
  // read duration till echo
  long duration = pulseIn(echoPin, HIGH, 60000);
  return duration ? (int)((duration*50L)/74L) : 10000;
}

void stopMoving()
{
  rm->run(RELEASE);
  lm->run(RELEASE);
  state = STOPPING;
}

void turn()
{
  static int dir = LEFT;
  static int turns = 0;
  
  if( state != TURNING)
  {
    rm->setSpeed( turnSpeed);
    lm->setSpeed( turnSpeed);
    rm->run(dir == LEFT ? FORWARD : BACKWARD );
    lm->run(dir == LEFT ? BACKWARD : FORWARD );
    state = TURNING;
    sr.setServoTargetAngle( 0, dir == LEFT ? SS_DEGREES(155) : SS_DEGREES(45) );
    sr.setServoMaxVelocity( 0, SS_DEGREES(60));
    if( ++turns == 3 )
    {
      // switch direction
       dir = (dir == LEFT ? RIGHT : LEFT);
       turns = 0; // reset count
    }
    
  }
}

void moveForward()
{
  if( state != MOVING)
  {
    // adjust motor speed so
    // you get a straight line 
    // while moving forward
    rm->setSpeed( moveSpeed+2);
    lm->setSpeed( moveSpeed);
    rm->run(FORWARD);
    lm->run(FORWARD);
    state = MOVING;
  }
}

// print debug info without spamming the output
void outputDebug( int distance )
{
  
  if( lastDistance != distance)
  {
    Serial.print("dist = ");
    Serial.print(distance);
    Serial.print("\" @ ");
    Serial.println(sr.getServoAngle(0)/1000); // we are assuming outputDebug() gets called shortly after readDistance();
    lastDistance = distance;
  }
  
  if( state != lastState )
  {
    Serial.println(stateName[state]);
    lastState = state;
  }
}

// the servo handler for Servorator
void update_servo( SS_Index index, SS_Angle angle, void *data)
{
  // SS_Angle is in 1000th of a degree
  //Serial.println(angle/1000);
  long time = trim.getServoPulseTime(index, angle);
  servo[index].writeMicroseconds( time );

}

void control_servos()
{
  if( state != TURNING )
  {
    
    SS_Angle target = sr.getServoTargetAngle(0);
    
    if( target != min_angle && target != max_angle )
    {
      // set target to the furthest angle
      if( target - min_angle > max_angle -target )
      {
        sr.setServoTargetAngle( 0, min_angle );
      }
      else
      {
        sr.setServoTargetAngle( 0, max_angle );
      }
    }
    else
    {
    
      SS_Angle angle = sr.getServoAngle(0); 

      if ( angle <= min_angle)
      { 
        sr.setServoTargetAngle(0, max_angle);
      } 
      else if ( angle >= max_angle)
      { 
        sr.setServoTargetAngle(0, min_angle);
      }
    } 
    sr.setServoMaxVelocity( 0, SS_DEGREES(120));
  }
  
  Serial.println(sr.getServoAngle(0)/1000);

  // sr.service() needs to be called regularly so that
  // the servos are updated via update_servo()
  sr.service();
}



