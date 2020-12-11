#include <NewPing.h>
#include <ViRHas.h>
#include <CytronMotorDriver.h>
#include <Encoder.h>


// SONARS

#define SONAR_NUM 4 
#define MAX_DISTANCE 300 // Max distance returned

unsigned long pingTimer[SONAR_NUM];

float dist[SONAR_NUM]; // Final distance communicated {F, R, L, B}
float cm[SONAR_NUM]; //Filtered sample
float cm_prec[SONAR_NUM]; // Filtered sample
float cm_raw[SONAR_NUM]; // Last raw
float cm_prec1[SONAR_NUM]; // second last raw etc..
float cm_prec2[SONAR_NUM];
float cm_prec3[SONAR_NUM];
float cm_prec4[SONAR_NUM];
float minDist = MAX_DISTANCE; //Support to calculate which sensor is working
int numActualData;

int minIndex;

bool backObstacle = true;  // Is there something behind my back?

String positionObject = "";

const int trigPin = 32;
const int echoPin = 30;

const int trigPin1 = 28;
const int echoPin1 = 26;

const int trigPin2 = 38;
const int echoPin2 = 36;

const int trigPin3 = 44;
const int echoPin3 = 42;

NewPing sonar[SONAR_NUM] = {
  NewPing(trigPin2, echoPin2, MAX_DISTANCE), //Front sonar
  NewPing(trigPin1, echoPin1, MAX_DISTANCE), //Right sonar
  NewPing(trigPin, echoPin, MAX_DISTANCE), //left sonar
  NewPing(trigPin3, echoPin3, MAX_DISTANCE), //back sonar  
  };



//MOTORS 
//input pins
#define _EP11 19
#define _EP12 18
#define _EP31 21
#define _EP32 20
#define _EP21 2
#define _EP22 3

//driver pins
#define _1_1A 8
#define _1_1B 7
#define _1_2A 5
#define _1_2B 4
#define _2_1A 12
#define _2_1B 11

//Maximum speed wantd
#define _MAX_SPEED 80 //cm/s
#define _MAX_ANGULAR 6//rad/s

//execution time
int i=1;
unsigned long StartTime = millis();
unsigned long CurrentTime = millis();
unsigned long ElapsedTime1 = 0;
unsigned long ElapsedTime2 = 0;
unsigned long ElapsedTime = 0;

//the different movemnts that the robot can performed.
//it is set to value between -1 and 1 the desired direction. Negative value, negative motion in the direction 
//selected. Positive value, positive movement in the desired direction

typedef struct motion_s {
  float strafe=0;
  float forward=0;
  float angular=0;
  int pan=90;
  int tilt=90;
} motion_t;

// Configure the motor driver.
CytronMD motor1(PWM_PWM, _1_1A, _1_1B); // PWM 2A = Pin 8, PWM 2B = Pin 7. Motor 1 : right robot
CytronMD motor2(PWM_PWM, _2_1A, _2_1B);   // PWM 1A = Pin 12, PWM 1B = Pin 11. Motor 2 : Atras
CytronMD motor3(PWM_PWM, _1_2A, _1_2B); // PWM 2A = Pin 5, PWM 2B = Pin 4. Motor 3 : left robot

//enable the encoders and set each eancoder of each sensor to which pin is connectes. take into coount that the
//order has to be Motor1,Motor2,Motor3. The order used in the motor has to joing with this one
Encoder ENCODER[] = { Encoder(_EP11, _EP12), Encoder(_EP21, _EP22), Encoder(_EP31, _EP32)};

//robot class
ViRHaS virhas = ViRHaS(motor1, motor2, motor3, ENCODER[0], ENCODER[1], ENCODER[2]);

//motion vectors (strafe, forward, angular, pan, tilt)
motion_t motion;

// in order to record the action that 
String movement = " ";
String data = " ";

int numberAction;

bool start = true;
bool moving = false;
int movingCode = 0;
int count = 0;

void setup() {

  virhas.setKpid(2, 0.6, 0.7);
  virhas.stop();
  Serial.begin(115200);
  
}

void loop() {
  
  // Reading data from SONAR
  sonarData();

   // I have cm[i] filled with information at this point
  if(cm[3] < 50) backObstacle = true;
  else backObstacle = false;
  
  minDist = MAX_DISTANCE;
  
  for(uint8_t i = 0; i < SONAR_NUM-1; i++){
    if(cm[i] < minDist){
      minDist = cm[i];
      minIndex = i;
      }
    }
  
  if(minIndex == 0) positionObject = "front";
  else if(minIndex == 1) positionObject = "right";
  else if(minIndex == 2) positionObject = "left";
  
  if(moving) movingCode = 1;
  else movingCode = 0;
 
  //Serial.print("Stream of Info-> Right: " + String(cm[1]) + " Front: " + String(cm[0]) + " Left: " + String(cm[2]) + " " + " Back: "+ String(cm[3]) + "\n");
  //delay(300);
 

  data = " ";
  
  // I send data to raspberry according to the speed of his while True
  
  if(Serial.available() > 0){
    data = Serial.readStringUntil('\n');
    Serial.print(positionObject + " " + String(minDist) + " " + String(movingCode) + "\n");
  }
  
  
  if(data != " " && data!= "ready") movement = data;
  
  numberAction = 0;
    
  if(movement == "move") forward();
  else if (movement == "rotateLeft") rotate(-1);
  else if (movement == "rotateRight") rotate(1);
  else if (movement == "scared") backward(0.3);
  else if (movement == "very_scared") backward(0.5);
  else if (movement == "excited_attract") sideStrafes();
  else if (movement == "interested_excited") smallRotations();
  else if (movement == "happy") bigRotations();
  else if (movement == "backForth") backForth();
  else{
    virhas.stop();
    movement = " ";
    moving = false;
  }
  

}


void sonarData(){
  
  for(uint8_t i = 0; i < SONAR_NUM; i++){
    
    cm_prec4[i] = cm_prec3[i];
    cm_prec3[i] = cm_prec2[i];
    cm_prec2[i] = cm_prec1[i];
    cm_prec1[i] = cm_raw[i];
    cm_prec[i] = cm[i];
  }
  
  for(uint8_t i = 0; i < SONAR_NUM; i++){
    cm_raw[i] = sonar[i].ping_cm();
    numActualData = 0;
    if(cm_raw[i] == 0){
      if(cm_prec1[i] + cm_prec2[i] + cm_prec3[i] + cm_prec4[i] == 0) cm[i] = MAX_DISTANCE;  //If it's 0 (ping not returned) and all the precedent are zero, then is a real zero
      else{
        numActualData = 0;
        if (cm_prec1[i] != 0) numActualData++;
        if (cm_prec2[i] != 0) numActualData++;
        if (cm_prec3[i] != 0) numActualData++;
        if (cm_prec4[i] != 0) numActualData++;
        cm[i] = (cm_prec1[i] + cm_prec2[i] + cm_prec3[i] + cm_prec4[i])/ numActualData; //If it's 0 but there i data in the "buffer" than the real result is the mean of the buffer
      }
    }
    else{
      if (cm_prec1[i] != 0) numActualData++;
      if (cm_prec2[i] != 0) numActualData++;
      if (cm_prec3[i] != 0) numActualData++;
      if (cm_prec4[i] != 0) numActualData++;
      cm[i] = (cm_raw[i] + cm_prec1[i] + cm_prec2[i] + cm_prec3[i] + cm_prec4[i])/(numActualData+1);      
    }
  }
}
void forward(){ // circa 20 cm forward
  if(minDist > 30){
    if(virhas.getPosX() < 20){
      moving = true;
      motion.strafe = 0;
      motion.forward = 0.3;
      motion.angular = 0;
      virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
      virhas.PIDLoop();
    }
    else{
      virhas.stop();
      movement = " ";
      moving = false;
    }
  }
  else{
    virhas.stop();
    movement = " ";
    moving = false;
  }
}

void backward(float speed){ // circa 40 cm backward at i speed
  if(!backObstacle){
    if(abs(virhas.getPosX()) < 40){
      moving = true;
      motion.strafe = 0;
      motion.forward = speed*(-1);
      motion.angular = 0;
      virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
      virhas.PIDLoop();
    }
    else{
      virhas.stop();
      movement = " ";
      moving = false;
    }
  }
  else{
    i = 1;
    while(abs(virhas.getPosTh()) < PI/6){
        moving = true;
        motion.strafe = 0;
        motion.forward = 0;
        motion.angular = 0.3*i;
        virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
        virhas.PIDLoop();
    }
    virhas.stop();
    while(numberAction < 5){ // KEEP THIS ODD
      i = i*(-1);
      while(abs(virhas.getPosTh()) < PI/3){
        moving = true;
        motion.strafe = 0;
        motion.forward = 0;
        motion.angular = i*(speed + 0.2);
        virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
        virhas.PIDLoop();
      }
      numberAction++; 
      virhas.stop();
    }
    virhas.stop();
    i = 1; 
    while(abs(virhas.getPosTh()) < PI/6){
        moving = true;
        motion.strafe = 0;
        motion.forward = 0;
        motion.angular = 0.3*i;
        virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
        virhas.PIDLoop();
    }
    virhas.stop();
    movement = " ";
    moving = false;    
  }
}

void rotate(int i){ // i is the direction of rotation
  
  while(abs(virhas.getPosTh()) < (PI/6)){
    moving = true;
    motion.strafe = 0;
    motion.forward = 0;
    motion.angular = i*0.3;
    virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
    virhas.PIDLoop();
  }
  virhas.stop();
  movement = " ";
  moving = false;
}

void sideStrafes(){
  int i = 1;
  while(abs(virhas.getPosY()) < 1.25){
      moving = true;
      motion.strafe = 0.8*i;
      motion.forward = 0;
      motion.angular = 0;
      virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
      virhas.PIDLoop();
  }
  virhas.stop();
  while(numberAction < 7){ // KEEP THIS ODD
    i = i*(-1);
    while(abs(virhas.getPosY()) < 2.5){
      moving = true;
      motion.strafe = 0.8*i;
      motion.forward = 0;
      motion.angular = 0;
      virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
      virhas.PIDLoop();
    }
    numberAction++; 
    virhas.stop();
  }
  virhas.stop();
  i = 1; 
  while(abs(virhas.getPosY()) < 1.25){
      moving = true;
      motion.strafe = 0.8*i;
      motion.forward = 0;
      motion.angular = 0;
      virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
      virhas.PIDLoop();
  }
  virhas.stop();
  movement = " ";
  moving = false;    
}

void backForth(){
  int i = 1;
  while(abs(virhas.getPosX()) < 1.25){
      moving = true;
      motion.strafe = 0;
      motion.forward = 0.8*i;
      motion.angular = 0;
      virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
      virhas.PIDLoop();
  }
  virhas.stop();
  while(numberAction < 7){ // KEEP THIS ODD
    i = i*(-1);
    while(abs(virhas.getPosX()) < 2.5){
      moving = true;
      motion.strafe = 0;
      motion.forward = 0.8*i;
      motion.angular = 0;
      virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
      virhas.PIDLoop();
    }
    numberAction++; 
    virhas.stop();
  }
  virhas.stop();
  i = 1; 
  while(abs(virhas.getPosX()) < 1.25){
      moving = true;
      motion.strafe = 0;
      motion.forward = 0.8*i;
      motion.angular = 0;
      virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
      virhas.PIDLoop();
  }
  virhas.stop();
  movement = " ";
  moving = false;    
}

void smallRotations(){
  int i = 1;
  while(abs(virhas.getPosTh()) < PI/12){
      moving = true;
      motion.strafe = 0;
      motion.forward = 0;
      motion.angular = 0.8*i;
      virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
      virhas.PIDLoop();
  }
  virhas.stop();
  while(numberAction < 7){ // KEEP THIS ODD
    i = i*(-1);
    while(abs(virhas.getPosTh()) < PI/6){
      moving = true;
      motion.strafe = 0;
      motion.forward = 0;
      motion.angular = 0.8*i;
      virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
      virhas.PIDLoop();
    }
    numberAction++; 
    virhas.stop();
  }
  virhas.stop();
  i = 1; 
  while(abs(virhas.getPosTh()) < PI/12){
      moving = true;
      motion.strafe = 0;
      motion.forward = 0;
      motion.angular = 0.8*i;
      virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
      virhas.PIDLoop();
  }
  virhas.stop();
  movement = " ";
  moving = false;    
}

void bigRotations(){
  int i = 1;
  while(abs(virhas.getPosTh()) < PI/6){
      moving = true;
      motion.strafe = 0;
      motion.forward = 0;
      motion.angular = 0.6*i;
      virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
      virhas.PIDLoop();
  }
  virhas.stop();
  while(numberAction < 2){ 
    i = i*(-1);
    while(abs(virhas.getPosTh()) < 2*PI){
      moving = true;
      motion.strafe = 0;
      motion.forward = 0;
      motion.angular = 0.6*i;
      virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
      virhas.PIDLoop();
    }
    numberAction++; 
    virhas.stop();
  }
  virhas.stop();
  i = -1; 
  while(abs(virhas.getPosTh()) < PI/6){
      moving = true;
      motion.strafe = 0;
      motion.forward = 0;
      motion.angular = 0.6*i;
      virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
      virhas.PIDLoop();
  }
  virhas.stop();
  movement = " ";
  moving = false;    
}
