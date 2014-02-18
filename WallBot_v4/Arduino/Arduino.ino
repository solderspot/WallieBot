// Copyright (c) 2014, Solder Spot
// All rights reserved.
// See license.txt 

// include needed libraries
 
#include <Wire.h>

typedef unsigned char ubyte;

//----------------------------------------
// Serial output config
//----------------------------------------

#define SERIAL_BAUD   9600

#define SENSOR_INFO    1
#define PID_INFO       0
#define MOTOR_INFO     0
#define STATE_INFO     0
#define BUTTON_INFO    0
#define TEST_ENCODERS  0
#define USE_ENCODERS   1
#define TEST_I2C       1

//----------------------------------------
// Defines
//----------------------------------------

#define LEFT  1
#define RIGHT 0
#define FORWARDS  1
#define BACKWARDS -1
#define USE_SERIAL  (SENSOR_INFO|PID_INFO  \
                    |MOTOR_INFO|STATE_INFO \
                    |TEST_ENCODERS|BUTTON_INFO)

//----------------------------------------
// Pin Assignments
//----------------------------------------

char lftApin      = A2;
char lftBpin      = A3;
char rhtApin      = A0;
char rhtBpin      = A1;
char rhtMtrDirPin = 7;
char lftMtrDirPin = 8;
char rhtMtrPWMPin = 9;
char lftMtrPWMPin = 10;
char buttonPin    = 12;
char ledPin       = 13;

//----------------------------------------
// Config
//----------------------------------------

int stopDistance        = 4;    // inches
int startDistance       = 11;   // inches
int turnSpeed           = 30;     // 0..127
int moveSpeed           = 50;     // 0..127
// motor config
int RMotorDirection     = BACKWARDS;
int LMotorDirection     = FORWARDS;
bool swapMotors         = false;
// servo config
int servoPWMMin         = 700;    // useconds
int servoPWMMax         = 2550;   // useconds
ubyte servoCenter       = 90;
ubyte sweepMin          = 45;
ubyte sweepMax          = 145;
ubyte leftTurnPos       = 45;
ubyte rightTurnPos      = 155; 
// velocities in 10th degrees per second
int inTurnRate  = 55*10;
int sweepRate   = 100*10;
// PID - use this make the left motor more
// powerful or less pwerful than the right motor.
// Value is as a percentage, i.e. 110 means left
// motor will be 10% faster than the right. 90
// would mean the left motor is 10% slower
int LMotorGain        = 100;
// use this to set default state of PID control
bool PIDenabled         = true;
bool pushToStart        = true;
bool pushToStop         = true;

//----------------------------------------
// Trinket I2C 
//----------------------------------------

#define TRINKET_ADDRESS    0x4

#define TRKT_STATUS           0
#define TRKT_SONIC_DISTANCE   1
#define TRKT_SERVO_MODE       2
#define TRKT_SERVO_ANGLE      3
#define TRKT_SERVO_NEW_MODE   4
#define TRKT_SERVO_NEW_ANGLE  5
#define TRKT_SERVO_SWEEP_MIN  6
#define TRKT_SERVO_SWEEP_MAX  7
#define TRKT_SERVO_PWM_MIN    8
#define TRKT_SERVO_PWM_MAX    10
#define TRKT_SERVO_VELOCITY   12
#define TRKT_SERVO_UPDATE     14
#define TRKT_TEST_COMMS       15
#define TRKT_STATUS_READY     1
#define TRKT_SERVO_SWEEP      0
#define TRKT_SERVO_FIXED      1


//----------------------------------------
// Data and Data Types 
//----------------------------------------

// Bot states
enum State
{
  IDLE,
  MOVING,
  STOPPING,
  TURNING,
  NUM_STATES
};

// for debug output
#if STATE_INFO
char *stateName[NUM_STATES] = 
{
  "idle",
  "moving",
  "stopping",
  "turning"
};
#endif

State state       = IDLE;
int lastDistance  = 0;
int lastState     = -1;
int adjustLMotor  = 0;
int adjustRMotor  = 0;
int LMotorSpeed   = 0;
int RMotorSpeed   = 0;


//----------------------------------------
// protos
//----------------------------------------
void setServoTargetAngle( ubyte angle );
void setServoSweep( ubyte min, ubyte max );
int getServoAngle( void );
void updateServo( void );
void set_motor( int index, char level );

//----------------------------------------
// setup
//----------------------------------------

void setup() 
{

  #if USE_SERIAL
  Serial.begin(SERIAL_BAUD);
  Serial.println("Wall Bot V4!");
  #endif

  Wire.begin();
  
  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin, HIGH);
  pinMode(ledPin, OUTPUT);

  pinMode( lftMtrDirPin, OUTPUT);
  pinMode( rhtMtrDirPin, OUTPUT);
  pinMode( lftMtrPWMPin, OUTPUT);
  pinMode( rhtMtrPWMPin, OUTPUT);
  
#if USE_ENCODERS
   // set up encoder
  setup_encoder(lftApin, lftBpin, rhtApin, rhtBpin);

#endif

  usePID(PIDenabled);

  waitForTrinket();
  setServoTargetAngle(servoCenter);
  setServoPWM(servoPWMMin, servoPWMMax);
  setServoMaxVelocity( inTurnRate );  
  updateServo();
  dumpTrinketRegs();
}

void dumpTrinketRegs()
{
  dumpReg( TRKT_STATUS, "Status" );
  dumpReg( TRKT_SONIC_DISTANCE, "Distance" );
  dumpReg( TRKT_SERVO_MODE, "Servo Mode" );
  dumpReg( TRKT_SERVO_ANGLE, "Servo Angle" );
  dumpReg( TRKT_SERVO_NEW_MODE, "Servo New Mode" );
  dumpReg( TRKT_SERVO_NEW_ANGLE, "Servo New Angle" );
  dumpReg( TRKT_SERVO_SWEEP_MIN, "Servo Sweep Min" );
  dumpReg( TRKT_SERVO_SWEEP_MAX, "Servo Sweep Max" );
  dumpReg2( TRKT_SERVO_PWM_MIN, "Servo PWM Min" );
  dumpReg2( TRKT_SERVO_PWM_MAX, "Servo PWM Max" );
  dumpReg2( TRKT_SERVO_VELOCITY, "Servo Velocity" );

}

void dumpReg( char reg, char *name )
{
  char val = -1;

  trktRegRead( reg, &val );

  Serial.print(name);
  Serial.print(": ");
  Serial.println((int) val);

}

void dumpReg2( char reg, char *name )
{
  int val = -1;

  trktRegRead2( reg, &val );

  Serial.print(name);
  Serial.print(": ");
  Serial.println(val);

}

//----------------------------------------
// loop - Main logic
//----------------------------------------

void loop() 
{

  #if TEST_I2C
  testI2C();
  #endif
  
  handleButtonPress();

  if( pushToStart )
  {
    // don't do anything till the button is pressed
    return;
  }
  
  // read distance from sensor
  int distance = readDistance();

  // output debug info
  outputDebug( distance);

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
      else
      {
        if ( LMotorSpeed < moveSpeed )
        {
          LMotorSpeed+=5;
          RMotorSpeed = LMotorSpeed;
          updateMotors();
        }
        #if !TEST_ENCODERS
          if ( PIDenabled )
          {
            driveStraight(); 
          }
        #endif
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
}

//----------------------------------------
// button logic
//----------------------------------------

void handleButtonPress(void)
{
  static char lastButton = HIGH;
  static char count = 0;           
    
  char button =  digitalRead( buttonPin );
    
  if( (button != lastButton) )
  {
    if(++count > 2) 
    {
      if( button == LOW )
      {
        #if BUTTON_INFO
          Serial.println("Button Pressed");
        #endif
        if( pushToStart )
        {
          pushToStart = false;
        }
        else if( pushToStop )
        {
          pushToStart = true;
          stopMoving();
          setServoTargetAngle(servoCenter);
          updateServo();
          state = IDLE;
        }
        else
        {
          usePID(!PIDenabled);
        }
      }
      lastButton = button;
      count = 0;
    }
  }
  else
  {
    count = 0;
  }
  
  #if TEST_ENCODERS && USE_ENCODERS
  {
    int16_t lft, rht;
    uint16_t ms;
    static int16_t lTotal = 0;
    static int16_t rTotal = 0;
    static int16_t lastlTotal = 0;
    static int16_t lastrTotal = 0;
    bool ok = get_ticks_since_last( &lft, &rht, &ms);
    lTotal += lft;
    rTotal += rht;
    
    if( lastlTotal != lTotal || lastrTotal != rTotal )
    {
      Serial.print("Encoders: lft = ");
      Serial.print(lTotal);
      Serial.print(" rht = ");
      Serial.print(rTotal);
      Serial.println( !ok ? " (error)" :"");
      lastlTotal = lTotal;
      lastrTotal = rTotal;
    }
  }
  #endif
}

//----------------------------------------
// 
//----------------------------------------

void usePID( bool enable )
{
  PIDenabled = enable;
  digitalWrite( ledPin, enable ? HIGH : LOW);
  resetPID();
}

//----------------------------------------
// Sensor
//----------------------------------------

// returns inches
int readDistance()
{
  ubyte dist;
  
  if( trktRegRead( TRKT_SONIC_DISTANCE, (char*)&dist))
  {
    return (int)dist;
  }

  return -1;
}

//----------------------------------------
// Test I2C
//----------------------------------------

#if TEST_I2C
int testI2C()
{
  static char counter = 0;

  if( trktRegWrite(TRKT_TEST_COMMS, counter ) )
  {
    char val;

    if(  trktRegRead(TRKT_TEST_COMMS, &val ) )
    {
      if( counter != val)
      {
        Serial.print("I2C error: count ");
        Serial.print((int)val);
        Serial.print(" should be ");
        Serial.println((int)counter);
      }
    }
    else
    {
      Serial.println("I2C error: could read from Trinket");
    }

    counter++;
  }
  else
  {
    Serial.println("I2C error: could write to Trinket");
  }

  static ubyte lastAngle = 0;
  ubyte val;

  if(trktRegRead( TRKT_SERVO_ANGLE, (char*) &val ))
  {
    if(  val != lastAngle)
    {
      lastAngle = val;
      Serial.print("Angle = ");
      Serial.println((int)val);
    }
  }
  else
  {
    Serial.println("I2C error: could not read angle");
  }

 }
#endif

//----------------------------------------
// Motor control
//----------------------------------------

void stopMoving()
{
  resetPID();
  LMotorSpeed = 0;
  RMotorSpeed = 0;
  adjustLMotor = 0;
  adjustRMotor = 0;
  updateMotors();
  state = STOPPING;
}

//----------------------------------------
// 
//----------------------------------------

void turn()
{
  static int dir = LEFT;
  static int turns = 0;
  
  if( state != TURNING)
  {
    int p = (dir == RIGHT ? 1 : -1);
    
    RMotorSpeed = -p*turnSpeed;
    LMotorSpeed = p*turnSpeed;
    updateMotors();
    state = TURNING;
    setServoTargetAngle( dir == LEFT ? leftTurnPos : rightTurnPos );
    setServoMaxVelocity( inTurnRate);
    updateServo();
    if( ++turns == 3 )
    {
      // switch direction
      dir = (dir == LEFT ? RIGHT : LEFT);
      turns = 0; // reset count
    }
  }
}

//----------------------------------------
// 
//----------------------------------------

void moveForward()
{
  if( state != MOVING)
  {
    LMotorSpeed = 0;
    RMotorSpeed = 0;
    adjustLMotor = 0;
    adjustRMotor = 0;
    #if !TEST_ENCODERS
    resetPID();
    #endif
    updateMotors();
    state = MOVING;
    setServoSweep( sweepMin, sweepMax );
    setServoMaxVelocity( sweepRate);
    updateServo();
  }
}

//----------------------------------------
// 
//----------------------------------------

void updateMotors()
{
  static int lastLSpeed = 0;
  static int lastRSpeed = 0;
  
  // LMotorGain is used to simulate the
  // left motor as being more or less powerful
  // than the right. We use this to test the PID
  // controler. You can set LMotorGain and
  // see how well PID can correct the motor
  // imbalance

  int lspeed = (((LMotorSpeed+adjustLMotor)*LMotorGain)/100)*LMotorDirection;
  int rspeed = (RMotorSpeed+adjustRMotor)*RMotorDirection;

  if ( swapMotors)
  {
    int temp = rspeed;
    rspeed = lspeed;
    lspeed = temp;
  }

  if (lastLSpeed != lspeed)
  {
    
    setMotor( lspeed, lftMtrDirPin, LEFT );

    #if MOTOR_INFO
    Serial.print("LeftMotor: ");
    Serial.print(lspeed);
    Serial.print(" (");
    Serial.print(LMotorSpeed);
    Serial.print(", ");
    Serial.print(adjustLMotor);
    Serial.println(")");
    #endif
    
    lastLSpeed = lspeed;
  }

  if (lastRSpeed != rspeed)
  {
    setMotor( rspeed, rhtMtrDirPin,  RIGHT  );

    #if MOTOR_INFO
    Serial.print("RightMotor: ");
    Serial.print(rspeed);
    Serial.print(" (");
    Serial.print(RMotorSpeed);
    Serial.print(", ");
    Serial.print(adjustRMotor);
    Serial.println(")");
    #endif

    lastRSpeed = rspeed;
  }

}

//----------------------------------------
// 
//----------------------------------------

void setMotor( int speed, int dirPin, int motorIndex )
{
  int dir = speed < 0 ? -1 : 1 ;

  speed *= dir;

  digitalWrite( dirPin, dir < 0 ? HIGH : LOW );
  int pwm = map( speed, 0, 127, 0, USE_ENCODERS ?  64 : 20000 );
#if MOTOR_INFO
  Serial.print("set_motor( ");
  Serial.print(motorIndex);
  Serial.print(", ");
  Serial.print(pwm);
  Serial.println(")");
#endif

#if USE_ENCODERS
  set_motor( motorIndex, pwm );
#endif
}

//----------------------------------------
// Debug
//----------------------------------------

// print debug info without spamming the output
void outputDebug( int distance )
{
  #if SENSOR_INFO
  if( lastDistance != distance)
  {
    Serial.print("dist = ");
    Serial.print(distance);
    Serial.print("\" @ ");
    Serial.println(getServoAngle()); 
    lastDistance = distance;
  }
  #endif
  
  #if STATE_INFO
  if( state != lastState )
  {
    Serial.print("State = ");
    Serial.println(stateName[state]);
    lastState = state;
  }
  #endif
}

//----------------------------------------
// Servo Control
//----------------------------------------

void setServoMaxVelocity( int vel )
{
  trktRegWrite2( TRKT_SERVO_VELOCITY, vel);
}

void setServoPWM( int minWidth, int maxWidth )
{
  trktRegWrite2( TRKT_SERVO_PWM_MIN, minWidth);
  trktRegWrite2( TRKT_SERVO_PWM_MAX, maxWidth);
}

void setServoTargetAngle( ubyte angle )
{
  trktRegWrite( TRKT_SERVO_NEW_MODE, TRKT_SERVO_FIXED);
  trktRegWrite( TRKT_SERVO_NEW_ANGLE, angle );
}

void setServoSweep( ubyte min, ubyte max )
{
  trktRegWrite( TRKT_SERVO_NEW_MODE, TRKT_SERVO_SWEEP);
  trktRegWrite( TRKT_SERVO_SWEEP_MIN, min);
  trktRegWrite( TRKT_SERVO_SWEEP_MAX, max);
}

int getServoAngle()
{
  ubyte angle;

  if (trktRegRead( TRKT_SERVO_ANGLE, (char*)&angle))
  {
    return (int) angle;
  }

  return -1;
}

void updateServo( )
{
  trktRegWrite( TRKT_SERVO_UPDATE, 1);
}


//----------------------------------------
// Trinket I/O
//----------------------------------------


void trktDelay()
{
  static unsigned long last = 0;

  long diff = 10 - (millis() - last);

  if( diff > 0 )
    delay(diff);

  last = millis();
}

void waitForTrinket()
{
  char status = 0;

  Serial.print("Waiting for Trinket..");

  while(true)
  {
    if( trktRegRead(TRKT_STATUS, &status) && status == TRKT_STATUS_READY)
    {
      break;
    }
    Serial.print(".");
    delay(100);
  }

  Serial.println("Ready.");

}

bool trktRegRead( char reg, char *value )
{
  char count = 5;
  while(  count-- > 0 )
  {
    trktDelay();
    Wire.beginTransmission(TRINKET_ADDRESS);
    Wire.write(reg);
    if( Wire.endTransmission() == 0)
    {
      if( Wire.requestFrom(TRINKET_ADDRESS, 1) == 1 )
      { 
        *value = Wire.read();
        return true;
      }
    }
  }
    return false;
}

bool trktRegRead2( char reg, int *value )
{
  char count = 5;
  while(  count-- > 0 )
  {
    trktDelay();
    Wire.beginTransmission(TRINKET_ADDRESS);
    Wire.write(reg);
    if( Wire.endTransmission() == 0)
    {
      if( Wire.requestFrom(TRINKET_ADDRESS, 2) == 2 )
      { 
        uint16_t hi = (uint16_t)((uint8_t)Wire.read());
        uint16_t lo = (Wire.read()&0xff);
        *value = (int)((hi<<8)|lo);
        return true;
      }
    }
  }
    return false;
}

bool trktRegWrite( char reg, char value )
{
  char count = 5;
  while(  count-- > 0 )
  {
    trktDelay();
    Wire.beginTransmission(TRINKET_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    if( Wire.endTransmission() == 0)
    {
      return true;
    }
  }
  return false;
}

bool trktRegWrite2( char reg, int value )
{
  char count = 5;
  while(  count-- > 0 )
  {
    trktDelay();
    Wire.beginTransmission(TRINKET_ADDRESS);
    Wire.write(reg);
    Wire.write(value>>8);
    Wire.write(value&0xff);
    if( Wire.endTransmission() == 0 )
    {
      return true;
    }
  }
  return false;
}

