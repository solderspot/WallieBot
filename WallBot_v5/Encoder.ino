// Copyright (c) 2014, Solder Spot
// All rights reserved. 
// See license.txt 

//----------------------------------------
//
//----------------------------------------

volatile int16_t en_lft_ticks = 0;
volatile int16_t en_rht_ticks = 0;
volatile bool en_error = false;
long en_lastCall = 0;
 
// hard coding the pins to port D  
char en_rApin = 2;
char en_rBpin = 3;
char en_lApin = 4;
char en_lBpin = 5; 

//----------------------------------------
//
//----------------------------------------

void setup_encoder()
{
  cli();
      PCMSK2 = ((1<<PCINT21)|(1<<PCINT20)|(1<<PCINT19)|(1<<PCINT18));
      PCICR = (1<<PCIE2);
      PCIFR = 0xFF;
  sei();
}

//----------------------------------------
//
//----------------------------------------

bool get_ticks_since_last( int16_t *lft, int16_t *rht, uint16_t *ms )
{
  cli();
  *lft = en_lft_ticks;
  *rht = en_rht_ticks;
  long now = millis();
  *ms = (uint16_t)(now - en_lastCall);
  en_lastCall = now;
  en_lft_ticks = en_rht_ticks = 0;
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
  en_lft_ticks = en_rht_ticks = 0;
  en_error = false;
  en_lastCall = millis();
  sei();
}

//----------------------------------------
//
//----------------------------------------

ISR(PCINT2_vect)
{

  static char lastLA = 0;
  static char lastLB = 0;
  static char lastRA = 0;
  static char lastRB = 0;

  en_process(en_lApin, en_lBpin, &lastLA, &lastLB, &en_lft_ticks);
  en_process(en_rApin, en_rBpin, &lastRA, &lastRB, &en_rht_ticks);
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

