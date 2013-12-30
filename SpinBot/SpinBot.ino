
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *lm = AFMS.getMotor(3);
Adafruit_DCMotor *rm = AFMS.getMotor(4);

// Doesn't get much simpler than this
void setup() 
{
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Spin Bot!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
 lm->setSpeed(50);
 lm->run(FORWARD);
 rm->setSpeed(50);
 rm->run(BACKWARD);
}

void loop() 
{
}
