/*
  MR001004.h - Library by Michele Bertoni
  This library can be used to control a motor,
  driven with MR001-004 drivers
  The library is based on DualMC33926MotorShield
  drivers library, for maintaining compatibility
  between Triskar and ViRHaS libraries.
 */

#include "MR001004.h"

// Constructors ////////////////////////////////////////////////////////////////

MR001004::MR001004()
{
  //Pin map
  _M1A = 4;
  _M1B = 5;
  _M1PWM = 6;
  _M2A = 7;
  _M2B = 8;
  _M2PWM = 9;
}

MR001004::MR001004(unsigned char M1A, unsigned char M1B, unsigned char M1PWM,
                   unsigned char M2A, unsigned char M2B, unsigned char M2PWM)
{
  //Pin map
  //PWM1 and PWM2 cannot be remapped because the library assumes PWM is on timer1
  _M1A = M1A;
  _M1B = M1B;
  _M1PWM = M1PWM;
  _M2A = M2A;
  _M2B = M2B;
  _M2PWM = M2PWM;   
}

MR001004::MR001004(unsigned char M1A, unsigned char M1B, unsigned char M1PWM)
{
  //Pin map
  //PWM1 and PWM2 cannot be remapped because the library assumes PWM is on timer1
  _M1A = M1A;
  _M1B = M1B;
  _M1PWM = M1PWM; 
}

// Public Methods //////////////////////////////////////////////////////////////
void MR001004::init()
{
// Define pinMode for the pins and set the frequency for timer1.

  pinMode(_M1A,OUTPUT);
  pinMode(_M1B,OUTPUT);
  pinMode(_M1PWM,OUTPUT);
  pinMode(_M2A,OUTPUT);
  pinMode(_M2B,OUTPUT);
  pinMode(_M2PWM,OUTPUT);
}

// Public Methods //////////////////////////////////////////////////////////////
void MR001004::init1M()
{
// Define pinMode for the pins and set the frequency for timer1.

  pinMode(_M1A,OUTPUT);
  pinMode(_M1B,OUTPUT);
  pinMode(_M1PWM,OUTPUT);
}

void MR001004::init2M()
{
// Define pinMode for the pins and set the frequency for timer1.

  init();
}

void MR001004::setM1Dir(unsigned char dir){
	switch(dir){
		case 0:
			digitalWrite(_M1A, 0);
			digitalWrite(_M1B, 0);
			break;
		case 1:
			digitalWrite(_M1A, 1);
			digitalWrite(_M1B, 0);
			break;
		case 2:
			digitalWrite(_M1A, 0);
			digitalWrite(_M1B, 1);
			break;
		default:
			digitalWrite(_M1A, 1);
			digitalWrite(_M1B, 1);
	}
}

void MR001004::setM2Dir(unsigned char dir){
	switch(dir){
		case 0:
			digitalWrite(_M2A, 0);
			digitalWrite(_M2B, 0);
			break;
		case 1:
			digitalWrite(_M2A, 1);
			digitalWrite(_M2B, 0);
			break;
		case 2:
			digitalWrite(_M2A, 0);
			digitalWrite(_M2B, 1);
			break;
		default:
			digitalWrite(_M2A, 1);
			digitalWrite(_M2B, 1);
	}
}

// Set speed for motor 1, speed is a number betwenn -255 and 255
void MR001004::setM1Speed(int speed)
{
  unsigned char direction = 0;
  if (speed > 0){
  	direction = 1;
  }
  else if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    direction = 2;  // Preserve the direction
  }

  if (speed > 255)  // Max PWM dutycycle
    speed = 255;
  setM1Dir(direction);
  analogWrite(_M1PWM, speed); // default to using analogWrite
}

// Set speed for motor 2, speed is a number betwenn -200 and 200
void MR001004::setM2Speed(int speed)
{
  unsigned char direction = 0;
  if (speed > 0){
  	direction = 1;
  }
  else if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    direction = 2;  // Preserve the direction
  }

  if (speed > 255)  // Max PWM dutycycle
    speed = 255;
  setM2Dir(direction);
  analogWrite(_M2PWM, speed); // default to using analogWrite
 
}

// Set speed for motor 1 and 2
void MR001004::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}
