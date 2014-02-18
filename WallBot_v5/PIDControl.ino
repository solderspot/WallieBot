// Copyright (c) 2014, Solder Spot
// All rights reserved. 
// See license.txt 

//----------------------------------------
// Config
//----------------------------------------

#define Kp            900L
#define Ki           1800L
#define Kd              0L
#define SYSTEM_BIAS   -185L

#define CSV_OUTPUT      0

#define MAX_ADJUSTMENT    100  

//----------------------------------------
// Data
//----------------------------------------

int16_t pid_lastErr;
int16_t pid_sumErrs;
uint16_t pid_time;
int16_t pid_total_lticks;
int16_t pid_total_rticks;

//----------------------------------------
//
//----------------------------------------

void resetPID()
{
  pid_total_lticks = 0;
  pid_total_rticks = 0;
  pid_lastErr = 0;
  pid_sumErrs = 0;
  adjustLMotor = adjustRMotor = 0;
  updateMotors();
  clear_ticks();
  pid_time = 0;
  #if CSV_OUTPUT && PID_INFO
  Serial.print("PID Reset: System Bias = ");
  Serial.println(SYSTEM_BIAS);
  Serial.println("Time, Interval, Left Ticks, Right Ticks, Error, Sum Erros, Adjust, Left, Adjust Right");
  #endif
}

//----------------------------------------
//
//----------------------------------------

void driveStraight()
{
  static int16_t lticks = 0, rticks = 0;
  static uint16_t ms = 0;
  int16_t dlticks, drticks, diff;
  int32_t delta;
  uint16_t dms;
  
  get_ticks_since_last( &dlticks, &drticks, &dms);

  lticks += dlticks;
  rticks += drticks;
  pid_total_lticks += dlticks;
  pid_total_rticks += drticks;
  ms += dms;
  pid_time += dms;

  if ( ms > 200 )
  {
    int16_t rdir = rticks < 0 ? -1 : 1;
    int16_t ldir = lticks < 0 ? -1 : 1;

    // make the values positive
    lticks *= ldir;
    rticks *= rdir;

    int16_t bias = (rticks*SYSTEM_BIAS)/10000L;
    diff = ((lticks  - rticks + bias )*100L)/ms;

    // we want the difference to be 0

    // track the integral 
    pid_sumErrs += diff;

    // get the differential
    delta = (int32_t) (diff - pid_lastErr);

    int16_t P = (int16_t) ((Kp*((int32_t)diff) + Ki*((int32_t)pid_sumErrs) + (Kd*delta))/1000L);

    pid_lastErr = diff;

    // a positive error means the left motor is 
    // turning more than the right so adjust 
    // each motor accordingly
    int16_t adjust = (P>>1);
    adjustLMotor -= adjust*ldir;
    adjustRMotor += adjust*rdir;

    // Put a limit on the total adjustment in case PID gets out of control
    constrain( adjustLMotor, -MAX_ADJUSTMENT, MAX_ADJUSTMENT);
    constrain( adjustRMotor, -MAX_ADJUSTMENT, MAX_ADJUSTMENT);

    #if PID_INFO
      #if CSV_OUTPUT && PID_INFO
        Serial.print(pid_time); Serial.print(", ");
        Serial.print(ms); Serial.print(", ");
        Serial.print(lticks); Serial.print(", ");
        Serial.print(rticks); Serial.print(", ");
        Serial.print(diff); Serial.print(", ");
        Serial.print(pid_sumErrs); Serial.print(", ");
        Serial.print(adjustLMotor); Serial.print(", ");
        Serial.print(adjustRMotor);
        Serial.println();
    #else
        Serial.print("DIFF = ");
        Serial.print(diff);
        Serial.print(" ERR = ");
        Serial.print(pid_sumErrs);
        Serial.print(" ADJ = (");
        Serial.print(adjustLMotor);
        Serial.print(", ");
        Serial.print(adjustRMotor);
        Serial.println(")");
      #endif
    #endif
    updateMotors();
    lticks = 0;
    rticks = 0;
    ms = 0;
  }
}

//----------------------------------------
// 
//----------------------------------------

#define WHEEL_DIAM    38.5f   // mm
#define TRACK_WIDTH   108.0f   // mm
#define TICKS_PER_REV 1204.0f
#define MM_PER_TICK (((WHEEL_DIAM*PI)/TICKS_PER_REV))
#define MM_TO_TICKS(A) ((float)(A))/MM_PER_TICK
#define MM_PER_DEGREES(D) ((TRACK_WIDTH*PI*(D))/360.0f)

int16_t pid_target_ticks;
int16_t pid_lmotor_speed = 0;
int16_t pid_rmotor_speed = 0;
int16_t pid_motor_step = 20;
uint32_t pid_last_motor_update;


void pid_move( int16_t mm)
{
  resetPID();
  pid_target_ticks = (int16_t) MM_TO_TICKS(mm);
  pid_lmotor_speed = pid_rmotor_speed = moveSpeed;
  pid_last_motor_update = millis();
  Serial.print("Moving: ");
  Serial.print(mm);
  Serial.print(" mm - ("); 
  Serial.print(pid_target_ticks);
  Serial.println(" ticks)");
}

//----------------------------------------
// 
//----------------------------------------

void pid_turn( int16_t degrees)
{
  resetPID();
  pid_target_ticks = (int16_t) MM_TO_TICKS( MM_PER_DEGREES(degrees));

  int16_t speed = degrees < 0 ? turnSpeed : -turnSpeed;

  pid_lmotor_speed = speed;
  pid_rmotor_speed = -speed;
  pid_last_motor_update = millis();

  Serial.print("Turning: ");
  Serial.print(degrees);
  Serial.print(" d - ("); 
  Serial.print(pid_target_ticks);
  Serial.println(" ticks)");
}

//----------------------------------------
// 
//----------------------------------------

bool pid_update_speed( int16_t *speed, int16_t *target, int16_t step, uint32_t now )
{
  uint32_t ms = now - pid_last_motor_update;

  step = (step*ms)/100;

  int16_t diff = *target - *speed;
  int16_t dir = diff < 0 ? -1 : 1;

  diff *= dir;

  step = step > diff ? diff : step;

  *speed += step*dir;
  return step != 0;
}

//----------------------------------------
// 
//----------------------------------------

bool pid_service_next( void )
{
  driveStraight();

  if( pid_total_lticks >= pid_target_ticks || pid_total_rticks >= pid_target_ticks)
  {
 
    return true;
  }
  uint32_t now = millis();
  bool change = pid_update_speed( &LMotorSpeed, &pid_lmotor_speed, pid_motor_step, now );
  change = pid_update_speed( &RMotorSpeed, &pid_rmotor_speed, pid_motor_step, now ) || change;

  if( change )
  {
    updateMotors();
  }
  return false;
}

