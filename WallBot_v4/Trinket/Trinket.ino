// Get this from https://github.com/rambo/TinyWire
#include <TinyWireS.h>
#include <avr/power.h>
#include "Trinket.h"
#include <Servo8Bit.h>

char trigPin = PB3;
char echoPin = PB4;
char servoPin = PB1;

//----------------------------------------
// 
//----------------------------------------

#define GET_REG(A)    i2c_regs[A]
#define GET_REG2(A)   (((uint16_t)(i2c_regs[A])<<8)|(uint16_t)(i2c_regs[(A)+1]))
#define SET_REG(A,V)  i2c_regs[A] = (uint8_t)(V);
#define SET_REG2(A,V) {i2c_regs[A] = (uint8_t)(((V)>>8)&0xff); i2c_regs[(A)+1] = (uint8_t)((V)&0xff);}

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

volatile uint8_t i2c_regs[TRKT_NUM_REGISTERS];

// Tracks the current register pointer position
volatile byte reg_position;
const byte reg_size = sizeof(i2c_regs);
const byte reg_size_lessone = reg_size-1;

long minPWM;
long maxPWM;
long servoWidth = 1500L;
long sweepMin;
long sweepMax;
long targetAngle;
long currentAngle = -1;
long angleVelocity;

Servo8Bit servo;


//----------------------------------------
//  Setup
//----------------------------------------

void setup()
{
 if (F_CPU == 16000000) 
 {
   // we are running at 16MHz
   clock_prescale_set(clock_div_1);
 }

  // inititalize registers/system
  SET_REG(TRKT_STATUS, TRKT_STATUS_INITIALIZING);
  SET_REG(TRKT_SERVO_NEW_MODE, TRKT_SERVO_FIXED );
  SET_REG(TRKT_SERVO_NEW_ANGLE, 45 );
  SET_REG2(TRKT_SERVO_PWM_MIN_HI, 500L );
  SET_REG2(TRKT_SERVO_PWM_MAX_HI, 2500L );
  SET_REG2(TRKT_SERVO_VELOCITY_HI, 60*10L );
  SET_REG(TRKT_SERVO_SWEEP_MIN, 45 );
  SET_REG(TRKT_SERVO_SWEEP_MAX, 135 );
  SET_REG(TRKT_SERVO_UPDATE, 1 );

   // setup pins
  pinMode( trigPin, OUTPUT );
  pinMode( echoPin, INPUT );
  pinMode( servoPin, OUTPUT );
  digitalWrite( servoPin, LOW );
  servo.attach( servoPin, 0, 3000 );
  
  update_servo();

  // Start I2C handling
  TinyWireS.begin( TRINKET_ADDRESS );
  TinyWireS.onReceive( receiveEvent );
  TinyWireS.onRequest( requestEvent );

  // let master know we are ready
  SET_REG(TRKT_STATUS, TRKT_STATUS_READY);

}

//----------------------------------------
// Loop
//----------------------------------------

void loop()
{
    /**
     * This is the only way we can detect stop condition (http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=984716&sid=82e9dc7299a8243b86cf7969dd41b5b5#984716)
     * it needs to be called in a very tight loop in order not to miss any (REMINDER: Do *not* use delay() anywhere, use tws_delay() instead).
     * It will call the function registered via TinyWireS.onReceive(); if there is data in the buffer on stop.
     */
    TinyWireS_stop_check();
    update_servo();
    update_sonic();
}

//----------------------------------------
// Servo Logic
//----------------------------------------

void update_servo()
{
  static long lastTime = 0;
  static long totalIntervals = 0;

  // check for any changes from master
  update_servo_params();

  long now = micros();
  long interval = now - lastTime;

  if( interval > 10000L)
  {
    // handle servo movement logic
    switch ( GET_REG( TRKT_SERVO_MODE) )
    {
      case TRKT_SERVO_SWEEP:
      {
        // make the servo sweep from side to side
        if( targetAngle != sweepMin && targetAngle != sweepMax )
        {
          // servo is not currently sweeping so
          // set target to the furthest angle from
          // our current position
          if( currentAngle - sweepMin > sweepMax - currentAngle )
          {
            targetAngle = sweepMin;
          }
          else
          {
            targetAngle = sweepMax;
          }
        }
        else
        {
          // detect the end of a sweep and start another
        
          if ( currentAngle <= sweepMin)
          { 
            targetAngle = sweepMax;
          } 
          else if ( currentAngle >= sweepMax)
          { 
            targetAngle = sweepMin;
          }
        }
        break;
      }
      case TRKT_SERVO_FIXED:
      {
          break;
      }
      default: break;
    }

    if( currentAngle >= 0 )
    {
      long diff = targetAngle - currentAngle;

      if ( diff && angleVelocity )
      {

        long dir = diff < 0 ? -1 : 1;
        long mag = diff*dir;
        long ms = (interval + totalIntervals)/1000L;
        long angleDelta = (angleVelocity*ms)/1000L;

        totalIntervals = angleDelta ? 0L : totalIntervals + interval;

        angleDelta = angleDelta > mag ? mag : angleDelta;
        currentAngle += angleDelta*dir;
      }
    }
    else
    {
      totalIntervals = 0;
      currentAngle = targetAngle;
    }
      
    servoWidth = minPWM + ((maxPWM-minPWM)*(currentAngle/1000))/180L;
    SET_REG(TRKT_SERVO_ANGLE, (currentAngle/1000L));
    lastTime = now;
  }


#if 1
  servo.writeMicroseconds( servoWidth );

#else
  {
    static bool servoHi = false;
    static long servoNext = 0;
    static long servoLast = 0;

    // update PWM signal
    servoNext -= (now - servoLast);
    if ( servoNext < 0 )
    {
        digitalWrite( servoPin, servoHi ? LOW : HIGH);
        servoNext = servoHi ? 20000L - servoWidth : servoWidth;
        servoHi = !servoHi;
    }

    servoLast = now;
  }
#endif
}


void update_servo_params()
{
  if ( GET_REG(TRKT_SERVO_UPDATE))
  {

    // check registers for new values
    maxPWM = GET_REG2(TRKT_SERVO_PWM_MAX_HI);
    minPWM = GET_REG2(TRKT_SERVO_PWM_MIN_HI);
    angleVelocity = (long)(GET_REG2(TRKT_SERVO_VELOCITY_HI))*100L;
    uint8_t mode = GET_REG(TRKT_SERVO_NEW_MODE);

    switch( mode )
    {
      case TRKT_SERVO_SWEEP :
      {
        sweepMin = GET_REG(TRKT_SERVO_SWEEP_MIN)*1000L;
        sweepMax = GET_REG(TRKT_SERVO_SWEEP_MAX)*1000L;

        break;
      }
      case TRKT_SERVO_FIXED :
      {
        targetAngle = GET_REG(TRKT_SERVO_NEW_ANGLE)*1000L;
        targetAngle = targetAngle < 0L ?  0 : targetAngle;
        targetAngle = targetAngle > 180000L ?  180000L : targetAngle;
        break;
      }
      default: break;
    }

    SET_REG(TRKT_SERVO_MODE, mode);
    SET_REG(TRKT_SERVO_UPDATE, 0);
  }
}

void delayMicros( long time )
{
  long start = micros();
  while( (micros() - start) < time )
  {
    TinyWireS_stop_check();
  }
}

//----------------------------------------
// Sensor Logic
//----------------------------------------

void update_sonic()
{
  static bool  sonicWaiting = false;
  static long triggerStart = 0;
  static bool sonicHigh;
  
  long now = micros();
  long duration = now - triggerStart;

  if( sonicWaiting )
  {
    if (!sonicHigh && digitalRead( echoPin) == HIGH)
    {
      triggerStart = now;
      sonicHigh = true;
      return;
    }

      
    if( !sonicHigh || digitalRead( echoPin) != LOW)
    {
      if (duration < 6000L)
      {
        return;
      }
    }
   
    uint8_t inches = ((duration*50L)/7400L);
    SET_REG(TRKT_SONIC_DISTANCE, inches );
    sonicWaiting = false;
    triggerStart = now;
  }

  if( duration > 50000L )
  {
     // reset trig pin
    digitalWrite(trigPin, LOW);
    delayMicros(2);
    // pulse trig pin
    digitalWrite(trigPin, HIGH);
    delayMicros(10); 
    digitalWrite(trigPin, LOW);
    sonicWaiting = true;
    sonicHigh = false;
    triggerStart = micros();
  }
}


//----------------------------------------
// I2C Handling Logic
//----------------------------------------

/**
 * This is called for each read request we receive, never put more than one byte of data (with TinyWireS.send) to the 
 * send-buffer when using this callback
 */
void requestEvent()
{  
    TinyWireS.send(i2c_regs[reg_position]);
    // Increment the reg position on each read, and loop back to zero
    reg_position++;
    if (reg_position >= reg_size_lessone)
    {
        reg_position = 0;
    }
}

/**
 * The I2C data received -handler
 *
 * This needs to complete before the next incoming transaction (start, data, restart/stop) on the bus does
 * so be quick, set flags for long running tasks to be called from the mainloop instead of running them directly,
 */
void receiveEvent(uint8_t howMany)
{
    if (howMany < 1)
    {
        // Sanity-check
        return;
    }
    if (howMany > TWI_RX_BUFFER_SIZE)
    {
        // Also insane number
        return;
    }

    reg_position = TinyWireS.receive();
    howMany--;
    if (!howMany)
    {
        // This write was only to set the buffer for next read
        return;
    }
    while(howMany--)
    {
        i2c_regs[reg_position] = TinyWireS.receive();
        reg_position++;
        if (reg_position >= reg_size_lessone)
        {
            reg_position = 0;
        }
    }
}


