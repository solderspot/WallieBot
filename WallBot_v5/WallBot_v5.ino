// Copyright (c) 2014, Solder Spot
// All rights reserved.
// See license.txt 

// include needed libraries
// standard arduino libs

// other libs
// https://github.com/solderspot/SS_Servorator
// https://github.com/solderspot/SS_ServoTrim

#include <SS_Servorator.h>
#include <SS_ServoTrim.h>
#include <ZumoMotors.h>
//#include <RemoteData.h>

//----------------------------------------
// Serial output config
//----------------------------------------

#define SERIAL_BAUD   9600

#define SENSOR_INFO     1
#define PID_INFO        0
#define MOTOR_INFO      0
#define STATE_INFO      0
#define BUTTON_INFO     0
#define TEST_ENCODERS   0
#define USE_PATHS       0
#define USE_RD          0
#define INVERT_SERVO    1

//----------------------------------------
// Defines
//----------------------------------------

#define NUM_SERVOS      1
#define LEFT            1
#define RIGHT           0
#define FORWARDS        1
#define BACKWARDS      -1
#define USE_SERIAL  (SENSOR_INFO|PID_INFO  \
                    |MOTOR_INFO|STATE_INFO \
                    |TEST_ENCODERS|USE_PATHS \
                    |USE_RD)
          
#define PTH_END         0
#define PTH_MOVE        1
#define PTH_TURN        2

int16_t pathSequence[] = 
{
  PTH_MOVE, 600, PTH_TURN, 90, 
  PTH_MOVE, 600, PTH_TURN, 90, 
  PTH_MOVE, 600, PTH_TURN, 90, 
  PTH_MOVE, 600, PTH_TURN, 90,
  PTH_END 
};

//----------------------------------------
// Pin Assignments
//----------------------------------------

char trigPin    = A0;
char echoPin    = A1;
char servoPin   = A2;
char buttonPin  = 12;
char ledPin     = 13;

//----------------------------------------
// Config
//----------------------------------------

int stopDistance        = 600;    // 100ths of inches
int startDistance       = 1000;   // 100ths of inches
int turnSpeed           = 70;     // 0..400
int moveSpeed           = 80;     // 0..400
// motor config
int RMotorDirection     = BACKWARDS;
int LMotorDirection     = FORWARDS;
bool swapMotors         = false;
// servo config
int servoPWMMin         = 700;    // useconds
int servoPWMMax         = 2500;   // useconds
SS_Angle servoCenter    = SS_DEGREES(90);
SS_Angle sweepMin       = SS_DEGREES(45);
SS_Angle sweepMax       = SS_DEGREES(130);
SS_Angle leftTurnPos    = SS_DEGREES(155);
SS_Angle rightTurnPos   = SS_DEGREES(45);
// velocities in degrees per second
SS_Velocity inTurnRate  = SS_DEGREES(60);
SS_Velocity sweepRate   = SS_DEGREES(100);
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
// Instance classes
//----------------------------------------

SS_Servorator sr(NUM_SERVOS);
SS_ServoTrim trim(NUM_SERVOS);
ZumoMotors motor;

//----------------------------------------
// Remote Data
//----------------------------------------

#if USED_RD

RD_BUTTON_DEF(pidEnable, "PID Enabled", PIDenabled )
RD_VGROUP_DEF(Settings, "Settings")

RD_DATA_START(RD_Data)
  RD_GROUP(Settings)
    RD_BUTTON(pidEnable)
  RD_GROUP_END
ED_DATA_END

RemoteData rd;

#endif // USE_RD

//----------------------------------------
// setup
//----------------------------------------

void setup() 
{

  #if USE_SERIAL
  Serial.begin(SERIAL_BAUD);
  Serial.println("Wall Bot V5!");
  #endif
  
  #if USE_RD
  rd.init( RD_Data, serial_handler);
  #endif
    
  servoInit();
  
  // register servo handler
  sr.setServoHandler( update_servo, NULL);

  // set initial position and speed of servo
  sr.setServoTargetAngle( 0, servoCenter); 
  sr.setServoMaxVelocity( 0, sweepRate );

  // set up trims so servo ranges are correct
  trim.setServoPulseRange( 0, servoPWMMin, servoPWMMax);

  // wait for servo to get into starting position
  while ( sr.getServoAngle(0) != servoCenter )
  {
    sr.service();
  }

  //delay a second so we can verify the servo is correctly centered
  // if not use the trim to adjust center point. 
  delay(1000);
  
  // setup sonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin, HIGH);
  pinMode(ledPin, OUTPUT);

  // set up encoder
  setup_encoder();
  
  usePID(PIDenabled);
  

}

#if USE_RD

void rd_output ( const char *data, int count )
{
  Serial.write(data, count);
}

// called after every loop() to handle serial events
void serialEvent() 
{
  while (Serial.available()) 
  {
    rd.input((char)Serial.read());
  }
}

#endif

//----------------------------------------
// loop - Main logic
//----------------------------------------

void loop() 
{
  
  handleButtonPress();

  if( pushToStart )
  {
    // don't do anything till the button is pressed
    return;
  }
  
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
      else
      {
        #if USE_PATHS
          update_path();
        #else
          if ( LMotorSpeed < moveSpeed )
          {
            LMotorSpeed+=20;
            RMotorSpeed = LMotorSpeed;
            updateMotors();
          }
          #if !TEST_ENCODERS
            if ( PIDenabled )
            {
              driveStraight(); 
            }
          #endif
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
  // small delay or sensor
  // will get over triggered
  delay(100);


}

void stop()
{
  pushToStart = true;
  stopMoving();
  state = IDLE;
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
          #if USE_PATHS
          init_path( pathSequence );
          #endif
        }
        else if( pushToStop )
        {
          stop();
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
  #if TEST_ENCODERS
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

//----------------------------------------
// Motor control
//----------------------------------------

void stopMoving()
{
  resetPID();
  stopMotors();
  state = STOPPING;
 }

void stopMotors()
{
  resetPID();
  LMotorSpeed = 0;
  RMotorSpeed = 0;
  adjustLMotor = 0;
  adjustRMotor = 0;
  updateMotors();

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
    sr.setServoTargetAngle( 0, dir == LEFT ? leftTurnPos : rightTurnPos );
    sr.setServoMaxVelocity( 0, inTurnRate);
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

  constrain( lspeed, -400, 400 );
  constrain( rspeed, -400, 400 );


  if ( swapMotors)
  {
    int temp = rspeed;
    rspeed = lspeed;
    lspeed = temp;
  }

  if (lastLSpeed != lspeed)
  {
    motor.setLeftSpeed(lspeed);
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
    motor.setRightSpeed( rspeed );

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
    Serial.println(sr.getServoAngle(0)/1000); // we are assuming outputDebug() gets called shortly after readDistance();
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

// the servo handler for Servorator
void update_servo( SS_Index index, SS_Angle angle, void *data)
{
  // SS_Angle is in 1000th of a degree
  #if INVERT_SERVO
  angle = SS_DEGREES(180) - angle;
  #endif
  long time = trim.getServoPulseTime(index, angle);
  servoSetPosition( time );
}

//----------------------------------------
// 
//----------------------------------------

void control_servos()
{
  if( state != TURNING )
  {

    // make the servo sweep from side to side
    
    SS_Angle target = sr.getServoTargetAngle(0);
    
    if( target != sweepMin && target != sweepMax )
    {
      // servo is not currently sweeping so
      // set target to the furthest angle from
      // our current position
      if( target - sweepMin > sweepMax -target )
      {
        sr.setServoTargetAngle( 0, sweepMin );
      }
      else
      {
        sr.setServoTargetAngle( 0, sweepMax );
      }
    }
    else
    {
    // detect the end of a sweep and start another
    
      SS_Angle angle = sr.getServoAngle(0); 

      if ( angle <= sweepMin)
      { 
        sr.setServoTargetAngle(0, sweepMax);
      } 
      else if ( angle >= sweepMax)
      { 
        sr.setServoTargetAngle(0, sweepMin);
      }
    }
    sr.setServoMaxVelocity( 0, sweepRate);
  }
  
  // sr.service() needs to be called regularly so that
  // the servos are updated via update_servo()
  sr.service();
}

