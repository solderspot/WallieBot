// Copyright (c) 2014, Solder Spot
// All rights reserved. 
// See license.txt 

//----------------------------------------
//
//----------------------------------------

volatile int16_t _lft_ticks = 0;
volatile int16_t _rht_ticks = 0;
volatile uint16_t _counter = 0;
volatile bool _error = false;
char _lApin;
char _lBpin;
char _rApin;
char _rBpin;

//----------------------------------------
//
//----------------------------------------

void setup_encoder( char lftPinA, char lftPinB, char rhtPinA, char rhtPinB )
{
  _init_pin( &_lApin, lftPinA);
  _init_pin( &_lBpin, lftPinB);
  _init_pin( &_rApin, rhtPinA);
  _init_pin( &_rBpin, rhtPinB);

  cli();
  // set timer2 interrupt at 1000 Hz
	// we are assuming a clk speed of 16MHz
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  OCR2A = 249;
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22);   
  TIMSK2 |= (1 << OCIE2A);
  sei();
}

//----------------------------------------
//
//----------------------------------------

void _init_pin( char *pin, char value)
{
  *pin = value;
  pinMode(value, INPUT);
  digitalWrite( value, HIGH);
}

//----------------------------------------
//
//----------------------------------------

bool get_ticks_since_last( int16_t *lft, int16_t *rht, uint16_t *ms )
{
  cli();
  *lft = _lft_ticks;
  *rht = _rht_ticks;
  *ms = _counter;
  _lft_ticks = _rht_ticks = _counter = 0;
  char error = _error;
  _error = false;
  sei();

  return !error;
}

//----------------------------------------
//
//----------------------------------------

void clear_ticks()
{
  cli();
  _lft_ticks = _rht_ticks = _counter = 0;
  _error = false;
  sei();
}

//----------------------------------------
//
//----------------------------------------

ISR(TIMER2_COMPA_vect)
{
  // this routine gets called once every 1 millisecond
  _counter++;

  static char lastLA = 0;
  static char lastLB = 0;
  static char lastRA = 0;
  static char lastRB = 0;

  _process(_lApin, _lBpin, &lastLA, &lastLB, &_lft_ticks);
  _process(_rApin, _rBpin, &lastRA, &lastRB, &_rht_ticks);
}

//----------------------------------------
//
//----------------------------------------

void _process( char Apin, char Bpin, char *lastA, char *lastB, volatile int16_t *ticks )
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
    _error = true;
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

