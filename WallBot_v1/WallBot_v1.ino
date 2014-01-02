// Copyright (c) 2013-2014, Solder Spot
// All rights reserved. 

// include needed libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>

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

int trigPin = 9;
int echoPin = 10;

State state = IDLE;
int lastDistance = 0;
int lastState = -1;
int stopDistance = 500;    // 100ths of inches
int startDistance = 1000;  // 100ths of inches
int turnSpeed = 50;        // 0..255
int moveSpeed = 90;        // 0..s55

// forward references
void outputDebug( int distance );
void stopMoving( void );
void turn( void );
void moveForward( void );

void setup() 
{
  // set up Serial library at 9600 bps
  Serial.begin(9600);
  Serial.println("Wall Bot!");
  
  // set up motors 
  AFMS.begin();
  lm->run(RELEASE);
  rm->run(RELEASE);
 
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
        delay(200); // turn a little more
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
    Serial.println("\"");
    lastDistance = distance;
  }
  
  if( state != lastState )
  {
    Serial.println(stateName[state]);
    lastState = state;
  }
}
