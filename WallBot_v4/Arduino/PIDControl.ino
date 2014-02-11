// Copyright (c) 2014, Solder Spot
// All rights reserved. 
// See license.txt 
#if USE_ENCODERS
//----------------------------------------
// Config
//----------------------------------------

#define Kp    0000L
#define Ki    000L
#define Kd    0L

//----------------------------------------
// Data
//----------------------------------------

int16_t pid_lastErr;
int16_t pid_sumErrs;

//----------------------------------------
//
//----------------------------------------

void resetPID()
{
  pid_lastErr = 0;
  pid_sumErrs = 0;
  adjustLMotor = adjustRMotor = 0;
  updateMotors();
  clear_ticks();
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
  ms += dms;

  if ( ms > 100 )
  {
    // we assume wheels are turning in the same direction
    int16_t dir = ( lticks < 0 || rticks < 0) ? -1 : 1;

    // make the values positive
    lticks *= dir;
    rticks *= dir;

    diff = lticks - rticks;

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
    int16_t ladjust = ((P+0)/2)*dir;
    int16_t radjust = (P/2)*dir;
    adjustLMotor -= ladjust;
    adjustRMotor += radjust;
    #if PID_INFO
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
    updateMotors();
    lticks = 0;
    rticks = 0;
    ms = 0;
  }
}
#else
void resetPID() {}
void driveStraight() {}

#endif
