
#ifndef __TRINKET_H
#define __TRINKET_H

#define TRINKET_ADDRESS 	0x4

enum TrinketRegisters
{
  TRKT_STATUS = 0,				// Read-only - See TrinketStatus
  TRKT_SONIC_DISTANCE =1,		// Read-only - current distance in inches
  TRKT_SERVO_MODE = 2,			// Read-only - current servo mode - see TrinketServoMode
  TRKT_SERVO_ANGLE = 3,			// Read-only - current servo angle - 0-180
  TRKT_SERVO_NEW_MODE = 4,		// Write-only - set new servo mode - See TrinketServoMode
  TRKT_SERVO_NEW_ANGLE = 5,		// Write-only - set new angle for TRKT_SERVO_FIXED
  TRKT_SERVO_SWEEP_MIN = 6, 	// Write-only - set min sweep angle 0-180
  TRKT_SERVO_SWEEP_MAX = 7,		// Write-only - set max sweep angle 0-180
  TRKT_SERVO_PWM_MIN_HI = 8,	// Write-only - set min PWM microseconds hi byte
  TRKT_SERVO_PWM_MIN_LO = 9,	// Write-only - set min PWM microseconds low byte
  TRKT_SERVO_PWM_MAX_HI = 10,	// Write-only - set max PWM microseconds hi byte
  TRKT_SERVO_PWM_MAX_LO = 11,	// Write-only - set max PWM microseconds low byte
  TRKT_SERVO_VELOCITY_HI = 12,	// Write-only - set velicoty hi byte
  TRKT_SERVO_VELOCITY_LO = 13,	// Write-only - set velicoty low byte
  TRKT_SERVO_UPDATE = 14,		// Write-only - set to non-zero value to force update
  TRKT_TEST_COMMS = 15,			// Used to test reliabalitiy of I2C		
  TRKT_NUM_REGISTERS = 16
};

enum TrinketStatus
{
	TRKT_STATUS_INITIALIZING = 0,		// Trinket is setting up 
	TRKT_STATUS_READY					// All systems are go
};

enum TrinketServoMode
{
	TRKT_SERVO_SWEEP = 0,				// 
	TRKT_SERVO_FIXED
};

#endif


