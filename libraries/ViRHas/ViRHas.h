/*
  Fork by Michele Bertoni of library Triskar.h - Library for controlling the Triskar robot.
  Created by Martino Migliavacca, August 2, 2013.
  Released into the public domain.
  Modification made by Matteo Lucchelli including
    - encoders reader
    - PID speed control
  - Odometry
  Modification made by Michele Bertoni:
    - added support for MR001-004 drivers
    - minor changes to make this library work with ViRHaS robot
*/

#ifndef ViRHaS_h
#define ViRHaS_h

#include "Arduino.h"
#include "CytronMotorDriver.h"
#include "Encoder.h"

#define NMOTOR 3
#define wheel_radius  3.5f //cm
#define robot_radius  12.5f  //cm

class ViRHaS
{
private:

  #define KP  0.35f;
  #define KI  0.8f;
  #define _MAX_DTH  30.0f     // Maximum wheel angular speed [rad/s]
  #define _MAX_SP   400.0f   // Maximum setpoint
  #define _SP_SCALE (_MAX_SP / _MAX_DTH)
  #define LOOPTIME        25                     // PID loop time
  double speed_req[3];         //SETPOINT
  double speed_act[3];                              // speed (actual value)
  int PWM_val[3];                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
  double last_error[3];
  double Iterm[3];
  float Kp;
  float Ki;
  float Kd;
  double posX;
  double posY;
  double posTh;
  double speedX;
  double speedY;
  double speedTh;

  unsigned long lastMilliLoop;
  unsigned long lastMillis[3];
  unsigned long deltaPid;

  CytronMD & _m1;
  CytronMD & _m2;
  CytronMD & _m3;
  Encoder & _e1;
  Encoder & _e2;
  Encoder & _e3;
  
  void getMotorRPM(long deltaT,int pos,int i);
  void getMotorRadS(long deltaT,int pos,int i);
  void getMotorCmS(long deltaT,int pos,int i);
  
  int updatePid(double targetValue, double currentValue, int i);
  void direct_kinematics(void);
  void makeOdometry(unsigned long int deltaT);

public:
  ViRHaS(CytronMD & m1,CytronMD & m2, CytronMD & m3,
      Encoder & e1, Encoder & e2, Encoder & e3);
  void run(float forward, float angular);
  void run2(float strafe, float forward, float angular);
  void runM(float m1, float m2, float m3);
  void setM1Speed(float m1);
  void setM2Speed(float m2);
  void setM3Speed(float m3);

  void PIDLoop();

  double getPosX();
  double getPosY();
  double getPosTh();

  void setPosX(double _posX);
  void setPosY(double _posY);
  void setPosTh(double _posTh);

  void setIterm(int i, double val);

  void setKp(double val);
  void resetKp();

  void setKi(double val);
  void resetKi();
    
  void setKpid(double val, double val1, double val2);

  void stop(void);
  void stop2(void);


};

#endif /* Triskar_h */
