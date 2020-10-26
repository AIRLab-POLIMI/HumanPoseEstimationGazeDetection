/*
  Fork by Michele Bertoni of library Triskar.cpp - Library for controlling the Triskar robot.
  Created by Martino Migliavacca, August 2, 2013.
  Released into the public domain.
  Modification made by Julian Angel
  -variables dth1, dth2 and dth2 now are global variables
  -add of new methods to get the points
  Modification made by Michele Bertoni
    -edited constructor and motor definitions, in order to make the code work with
     motor drivers "MR001-004"
*/

#include "ViRHaS.h"


ViRHaS::ViRHaS(CytronMD & m1, CytronMD & m2,CytronMD & m3, Encoder & e1, Encoder & e2, Encoder & e3)
: _m1(m1), _m2(m2), _m3(m3), _e1(e1),_e2(e2),_e3(e3) {
  //_m1m2.init2M();
  _m1.setSpeed(0);
  _m2.setSpeed(0);
  //_m3.init1M();
  _m3.setSpeed(0);
   posX=0;
   posY=0;
   posTh=0;
   speedX=0;
   speedY=0;
   speedTh=0;

   speed_req[0] = 0.0;                      //SETPOINT
   speed_req[1] = 0.0;
   speed_req[2] = 0.0;

   speed_act[0] = 0.0;                              // speed (actual value)
   speed_act[1] = 0.0;
   speed_act[2] = 0.0;

   PWM_val[0] = 0.0;
   PWM_val[1] = 0.0;
   PWM_val[2] = 0.0;

   last_error[0]=0.0;
   last_error[1]=0.0;
   last_error[2]=0.0;
   Iterm[0]=0.0;
   Iterm[1]=0.0;
   Iterm[2]=0.0;

   //Kp = KP;
   //Ki = KI;
   //Kd = KD;

   lastMilliLoop=0;
   lastMillis[0]=0;
   lastMillis[1]=0;
   lastMillis[2]=0;
   deltaPid=0;
  
}

void ViRHaS::PIDLoop(){
  deltaPid=millis()-lastMilliLoop;
  if(deltaPid >= LOOPTIME)   {// enter tmed loop
     lastMilliLoop=millis();
     long deltaT;
     long ActualPos[3];
     ActualPos[0]=_e1.read();
     deltaT=millis()-lastMillis[0];
     getMotorRadS(deltaT,ActualPos[0],0);// calculate speed
     lastMillis[0] = millis();

     ActualPos[1]=_e2.read();
     deltaT=millis()-lastMillis[1];
     getMotorRadS(deltaT,ActualPos[1],1);
     lastMillis[1] = millis();

     ActualPos[2]=_e3.read();
     deltaT=millis()-lastMillis[2];
     getMotorRadS(deltaT,ActualPos[2],2);
     lastMillis[2] = millis();                                       // calculate speed,

     PWM_val[0]= updatePid(speed_req[0], speed_act[0], 0);           // compute PWM value
     PWM_val[1]= updatePid(speed_req[1], speed_act[1] ,1);
     PWM_val[2]= updatePid(speed_req[2], speed_act[2] ,2);
     _m1.setSpeed(PWM_val[0]);
     _m2.setSpeed(PWM_val[1]);
     _m3.setSpeed(PWM_val[2]);

     direct_kinematics();
     makeOdometry(deltaPid);

  }

}

void ViRHaS::run(float forward, float angular) {

  #define m1_R     (-1.0f / wheel_radius)
  #define mL_R     (-robot_radius / wheel_radius)
  #define C60_R    (0.500000000f / wheel_radius)   // cos(60°) / R
  #define C30_R    (0.866025404f / wheel_radius)   // cos(30°) / R


    //speed scomposition
    const float dy12 = C30_R * forward;
    const float dthz123 = mL_R * angular;
    // Wheel angular speeds
    speed_req[0] = + dy12 + dthz123;
    speed_req[1] = + dthz123;
    speed_req[2] = - dy12 + dthz123;

}

void ViRHaS::run2(float strafe, float forward, float angular) {

  #define m1_R     (-1.0f / wheel_radius)
  #define mL_R     (-robot_radius / wheel_radius)
  #define C60_R    (0.500000000f / wheel_radius)   // cos(60°) / R
  #define C30_R    (0.866025404f / wheel_radius)   // cos(30°) / R


    const float dx12 = C60_R * strafe;
  const float dy12 = C30_R * forward;
  const float dthz123 = mL_R * angular;

  speed_req[0] = dx12 + dy12 + dthz123; //motore anteriore dx
  speed_req[1] = m1_R * strafe + dthz123; // motore posteriore
  speed_req[2] = dx12 - dy12 + dthz123; //motore anteriore sx
//  _m1.setSpeed(speed_req[0]);
//  _m2.setSpeed(speed_req[1]);
//  _m3.setSpeed(speed_req[2]);
    
}

void ViRHaS::stop(void){
  _m1.setSpeed(0);
  _m2.setSpeed(0);
  _m3.setSpeed(0);
  for(int i=0;i<NMOTOR;i++){
    speed_act[i]=0;
    speed_req[i]=0;
    Iterm[i]=0;
  }
  posX=0;
  posY=0;
  posTh=0;
}

void ViRHaS::stop2(){
  _m1.setSpeed(0);
  _m2.setSpeed(0);
  _m3.setSpeed(0);
  for(int i=0;i<NMOTOR;i++){
    speed_act[i]=0;
    speed_req[i]=0;
  }
}

void ViRHaS::runM(float m1, float m2, float m3){
    // Motor setpoints
  speed_req[0] = m1;
  speed_req[1] = m2;
  speed_req[2] = m3;
}
void ViRHaS::setM1Speed(float m1){
   speed_req[0] = m1;
}
void ViRHaS::setM2Speed(float m2){
  speed_req[1] = m2;
}
void ViRHaS::setM3Speed(float m3){
  speed_req[2] = m3;
}

//determina velocità in cm/s
void ViRHaS::direct_kinematics(void){

     speedX = 0.5*wheel_radius*(speed_act[0] - speed_act[2])/cos((float)PI/6.0f);
     speedY = wheel_radius*(speed_act[2] + speed_act[0]-2.0*speed_act[1])/3.0;
     speedTh= wheel_radius*(speed_act[2]+speed_act[0]+speed_act[1])/3.0/robot_radius;

}

void ViRHaS::makeOdometry(unsigned long int deltaT){
   double delta_x = (speedX * cos(posTh) - speedY * sin(posTh)) * deltaT/1000.0;
     double delta_y = (speedX * sin(posTh) + speedY * cos(posTh)) * deltaT/1000.0;
     double delta_th = speedTh * deltaT/1000.0;

  posX+=delta_x;
  posY+=delta_y;
  posTh+=delta_th;

}

int ViRHaS::updatePid(double targetValue, double currentValue, int i)   {// compute PWM value
  double pidTerm =0;                                                            // PID correction
  double error=0;
  error = targetValue - currentValue;
  Iterm[i] += error*Ki;
  if(Iterm[i]>255) Iterm[i]=255;
  else if(Iterm[i]<-255) Iterm[i]=-255;
  double deltaError = error - last_error[i];
  pidTerm = (Kp * error) + (Iterm[i]) + (Kd * deltaError);
  last_error[i] = error;
  int output=constrain(int(pidTerm), -255, 255);
  return output;
}


void ViRHaS::getMotorCmS(long deltaT,int pos,int i)  {                                                        // calculate speed, volts and Amps
static int countAnt[3] = {0,0,0};          // last count
speed_act[i] = ((pos - countAnt[i])*(2.0f*(float)PI*wheel_radius)*1000.0f)/(deltaT*1920L);         // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
 countAnt[i] = pos;
}



void ViRHaS::getMotorRadS(long deltaT,int pos,int i)  {                                                        // calculate speed, volts and Amps
static int countAnt[3] = {0,0,0};                                                   // last count
 speed_act[i] =(float) ((pos - countAnt[i])*(1000L/deltaT)*2.0f*(float)PI)/(1920.0f);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
 countAnt[i] = pos;
}

void ViRHaS::getMotorRPM(long deltaT,int pos,int i)  {                                                        // calculate speed, volts and Amps
static int countAnt[3] = {0,0,0};                                                   // last count
 speed_act[i] = ((pos - countAnt[i])*(60L*(1000L/deltaT)))/(1920);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
 countAnt[i] = pos;
}

double ViRHaS::getPosX(){
  return posX;
}
double ViRHaS::getPosY(){
  return posY;
}
double ViRHaS::getPosTh(){
  return posTh;
}

void ViRHaS::setPosX(double _posX){
  posX=_posX;
}
void ViRHaS::setPosY(double _posY){
  posY=_posY;
}
void ViRHaS::setPosTh(double _posTh){
  posTh=_posTh;
}

void ViRHaS::setIterm(int i, double val){
  Iterm[i]=val;
}

void ViRHaS::setKp(double val){
  Kp=val;
}

void ViRHaS::setKi(double val){
  Ki=val;
}

void ViRHaS::setKpid(double val, double val1, double val2){
  Kp=val;
  Ki=val1;
  Kd=val2;
}

void ViRHaS::resetKp(){
  Kp=KP;
}

void ViRHaS::resetKi(){
  Ki=KI;
}
