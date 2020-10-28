#include <NewPing.h>
#include <ViRHas.h>
#include <CytronMotorDriver.h>
#include <Encoder.h>


// SONARS

#define SONAR_NUM 4 
#define MAX_DISTANCE 300 // Max distance returned
#define PING_INTERVAL 33 // ms between pings from each sensor
#define BUFF_LEN 10 // Length of the buffer to clean the data

unsigned long pingTimer[SONAR_NUM];

float dist[SONAR_NUM];// Final distance communicated {F, R, L, B}
float cm[SONAR_NUM]; //To store filtered actual ping distance
float cm_prec[SONAR_NUM]; //To store filtered previous ping distance
float minDist = MAX_DISTANCE; //Support to calculate which sensor is working

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

float actualPosX = 0;
float actualPosY = 0;
float actualPosTh = 0;

bool start = true;
bool moving = false;
int count = 0;

void setup() {

  virhas.setKpid(2.0, 0.6, 0.5);
  virhas.stop();
  sonarSetup();
  
}

void loop() {
  
  // Reading data from SONAR
  
  for(uint8_t i = 0; i < SONAR_NUM; i++){
    cm_prec[i] = cm[i];
    }
  
  for(uint8_t i = 0; i < SONAR_NUM; i++){
    if(millis() >= pingTimer[i]){
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      dist[i] = sonar[i].ping_median(BUFF_LEN);
      cm[i] = sonar[i].convert_cm(dist[i]);  // I have cm[i] filled with information at this point
      if(cm[i] == 0){
        cm[i] = cm_prec[i];
        }
      }
    }
  
  if(cm[3] < 30) backObstacle = true;
  else backObstacle = false;
  
  minDist = MAX_DISTANCE;
  
  for(uint8_t i = 0; i < SONAR_NUM-1; i++){
    if(cm[i] < minDist){
      minDist = cm[i];
      minIndex = i;
      }
    }
  
  if(minIndex == 0) positionObject = "Front";
  else if(minIndex == 1) positionObject = "Right";
  else if(minIndex == 2) positionObject = "Left";
  
  if(!moving){
    Serial.print("Stream of Info-> Right: " + String(cm[1]) + " Front: " + String(cm[0]) + " Left: " + String(cm[2]) + " " + "POSITION: "+positionObject);
    virhas.stop();
  }
  
  if(Serial.available() > 0){    
    data = Serial.readStringUntil('\n');
  }
  else data = " ";
  
  if(data != " ") movement = data;
    
  if(movement == "move") forward();
  else{
    virhas.stop();
    movement = " ";
    moving = false;
  }
  

}

void sonarSetup() {
  
  Serial.begin(115200);
  pingTimer[0] = millis() + 75;
  for(uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i-1] + PING_INTERVAL;
  
}

void forward(){
  if(minDist > 30){
    while(actualPosX < 20){
      moving = true;
      actualPosY = virhas.getPosY();
      actualPosX = virhas.getPosX();
      Serial.print("\n Pos Y: " + String(actualPosY) + " Pos X: " + String(actualPosX));
      motion.strafe = 0;
      motion.forward = 0.5;
      motion.angular = 0;
      virhas.run2(motion.strafe*_MAX_SPEED, motion.forward*_MAX_SPEED, motion.angular*_MAX_ANGULAR);
      virhas.PIDLoop();
    }
    virhas.stop();
    movement = " ";
    moving = false;
  }
}
