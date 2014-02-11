// Copyright (c) 2014, Solder Spot
// All rights reserved. 
// See license.txt 

#if USE_ENCODERS
#define TICK_HZ   16000L
#define CYCLE_HZ  250L
#define CYCLE_TICKS (TICK_HZ/CYCLE_HZ)

//----------------------------------------
//
//----------------------------------------

// encoder data
volatile int16_t en_lft_ticks = 0;
volatile int16_t en_rht_ticks = 0;
volatile uint16_t en_counter = 0;
volatile bool en_error = false;
char en_lApin;
char en_lBpin;
char en_rApin;
char en_rBpin;

// motor data
// each tick is 62.5 usec
// 64 ticks per cycle
struct MotorData
{
     char   on_ticks;
     volatile bool      on;
     volatile char   ticks_left;
     char     pin;
};

MotorData x_motor[2];
void init_motor( MotorData *m, char pin );
void update_motor( MotorData *m );

//----------------------------------------
//
//----------------------------------------

void setup_encoder( char lftPinA, char lftPinB, char rhtPinA, char rhtPinB )
{
  en_init_pin( &en_lApin, lftPinA);
  en_init_pin( &en_lBpin, lftPinB);
  en_init_pin( &en_rApin, rhtPinA);
  en_init_pin( &en_rBpin, rhtPinB);

  init_motor( &x_motor[LEFT], 10 );
  init_motor( &x_motor[RIGHT], 9 );
    
  cli();
  // set timer2 interrupt at 16000 Hz
  // we are assuming a clk speed of 16MHz
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  OCR2A = 124;
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS21);   
  TIMSK2 |= (1 << OCIE2A);
  sei();
}

//----------------------------------------
//
//----------------------------------------

void en_init_pin( char *pin, char value)
{
  *pin = value;
  pinMode(value, INPUT);
  digitalWrite( value, LOW);
}

//----------------------------------------
//
//----------------------------------------

bool get_ticks_since_last( int16_t *lft, int16_t *rht, uint16_t *ms )
{
  cli();
  *lft = en_lft_ticks;
  *rht = en_rht_ticks;
  *ms = en_counter>>4;
  en_lft_ticks = en_rht_ticks = en_counter = 0;
  char error = en_error;
  en_error = false;
  sei();

  return !error;
}

//----------------------------------------
//
//----------------------------------------

void clear_ticks()
{
  cli();
  en_lft_ticks = en_rht_ticks = en_counter = 0;
  en_error = false;
  sei();
}

//----------------------------------------
//
//----------------------------------------

ISR(TIMER2_COMPA_vect)
{
  en_counter++;

  static char lastLA = 0;
  static char lastLB = 0;
  static char lastRA = 0;
  static char lastRB = 0;

  en_process(en_lApin, en_lBpin, &lastLA, &lastLB, &en_lft_ticks);
  en_process(en_rApin, en_rBpin, &lastRA, &lastRB, &en_rht_ticks);
  update_motor( &x_motor[LEFT] );
  update_motor( &x_motor[RIGHT] );
}

//----------------------------------------
//
//----------------------------------------

void en_process( char Apin, char Bpin, char *lastA, char *lastB, volatile int16_t *ticks )
{
  char A = (digitalRead( Apin) == HIGH) ? 1 : 0;
  char B = (digitalRead( Bpin) == HIGH) ? 1 : 0;
  char lA = *lastA;
  char lB = *lastB;
  char dA = A!=lA;
  char dB = B!=lB;

  if( dA && dB )
  {
    // both should not change at the same time
    en_error = true;
  }
  else if ( dA || dB )
  {
    if (A^lB) 
    {
      *ticks += 1;
    }
    else if(B^lA)
    {
      *ticks -= 1;
    }
  }
  *lastA = A; 
  *lastB = B;
}

void init_motor( MotorData *m, char pin )
{
  m->on_ticks = 0;
  m->on = false;
  m->ticks_left = CYCLE_TICKS;
  m->pin = pin;
}

void set_motor( int index, char level )
{
  x_motor[index].on_ticks = level;
}

void update_motor( MotorData *m )
{
  if ( --(m->ticks_left) < 0 )
  {
    if ( m->on )
    {
        // turn off?
         m->on = !(m->ticks_left =  CYCLE_TICKS - m->on_ticks);
        if( !m->on )
        {
          digitalWrite( m->pin, LOW );
        }
    }
    else
    {
        // turn on?
        m->on = (m->ticks_left =  m->on_ticks) != 0;
        if( m->on )
        {
          digitalWrite( m->pin, HIGH );
        }
    }
  }
}

#endif
