/*
  MR001004.h - Library by Michele Bertoni
  This library can be used to control a motor,
  driven with MR001-004 drivers
  The library is based on DualMC33926MotorShield
  drivers library, for maintaining compatibility
  between Triskar and ViRHaS libraries.
 */

#ifndef MR001004_h
#define MR001004_h

#include <Arduino.h>

class MR001004
{
  public:  
    // CONSTRUCTORS
    MR001004(); // Default pin selection.
    MR001004(unsigned char M1A, unsigned char M1B, unsigned char M1PWM,
             unsigned char M2A, unsigned char M2B, unsigned char M2PWM); // User-defined pin selection. 
    MR001004(unsigned char M1A, unsigned char M1B, unsigned char M1PWM); //only one motor

    // PUBLIC METHODS
    void init(); // Initialize TIMER 1, set the PWM to 20kHZ. 
	  void init1M();
	  void init2M();
	  void setM1Dir(unsigned char dir); //0-X 1-CLOCKWISE 2-COUNTERCLOCKWISE
	  void setM2Dir(unsigned char dir); //0-X 1-CLOCKWISE 2-COUNTERCLOCKWISE
    void setM1Speed(int speed); // Set speed for M1.
    void setM2Speed(int speed); // Set speed for M2.
    void setSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
    
  private:
    unsigned char _M1A;
    unsigned char _M1B;
    unsigned char _M2A;
    unsigned char _M2B;
    unsigned char _M1PWM;
    unsigned char _M2PWM;
};

#endif
