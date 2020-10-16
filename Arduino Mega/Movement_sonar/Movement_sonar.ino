/*****************************************************************************************************************
 * Description: This code is the main code of the three scripts that are in this project. These three codes      *
 * enables the Arduino mega managing three main functions:                                                       *
 * - manage sensors in order to detect the distance and the position of an object in the environmen              *
 * - Enable the comunication with the Raspberry board in order to know the action to be performed                *
 * - Depending on the action, decide the movement to perform and control it                                      *
 * ***************************************************************************************************************
 * Author: Ane San Martin Igarza                                                                                 *
 * ***************************************************************************************************************
 * Last Update: 23/06/2020                                                                                       *
 * **************************************************************************************************************/
#include <NewPing.h>
#include <ViRHas.h>
#include "CytronMotorDriver.h"
#include <Encoder.h>

//SONARS
// Sonar characteristics and parameters
#define SONAR_NUM     4 // Number or sensors.
#define MAX_DISTANCE 400 // Max distance in cm.
#define PING_INTERVAL 33 // Milliseconds between pings.
#define THRESHOLD_MEAN 30 // the maximum distance between means in order to make mean between the previous and the 
                          //current state or just use the new mean

unsigned long pingTimer[SONAR_NUM]; // When sonar each pings.
unsigned int cm[SONAR_NUM]; // Store ping distances.
uint8_t currentSensor = 0; // Which sensor is active.

uint16_t FAR_DISTANCE = 150;// the maximum distance in cm wanted inorde to start the interaction with a child
static const uint16_t VERYCLOSE_DISTANCE = 55;// the closest that the robot can be from the user in cm

// variables for sonnar management
#define COUNTER 1   //when targetPos reach COUNTER(right) or -COUNTER(left) the robot is sure about where obstacle 
                    //is(to avoid sonar false readings)
#define MAX_COUNTER 2*COUNTER   //maximum COUNTER value
#define MAX_DIFERENCE 20 // maximum diference between the previos obstacle distance to the new one to considere it
                        //as the same obstacle
#define MEAN_COUNT 300  // number of iterations until the meand of the data is done

//all the diustances at the beginning will be taken as the farer as possible
float previous_distance = 400;
float actual_distance = 400;// real actual distance
float actual_distance_mean = 400; // mean done with 300 iterations distances
float old_dist = 400;// distance from the previous mean

// to detect the user's behaviour
String movement_direction = "stopped"; // In order to detect in the direction that the user is moving
String previous_movement_direction = ""; 

// For the mean value of the distance
float distance_far = 0; 
float distance_right = 0;//distance detected from the right sensor
float distance_left = 0;//distance detected from the left sensor
float distance_front = 0;//distance detected from the front sensor

float dist_dif = 0;//distnace difference between the previous mean and the current one

bool no_obstacle = true;
int targetPos = 0;
bool new_target = false;

//Detection of obstacles by the back sensor. The back obstacle detection is just for stopping a movement if it gets
//close to one 
bool back_obstacle = false;
bool in_back = false;

bool send_data = false;
bool send_data_mov = false;

// when action is being performed it will not be sent data to Raspberry, just a word that says :"busy"
bool action_performing = false;

//counts used in the mean function in order to filter and know whereis in reality the obstacle
int count = 0;  // instead of using the interaction of each interaction it will be make a mean in order to know 
                // the position of the object, more accuratCe
int countnone = 0;
int countfront = 0;
int countright = 0;
int countleft = 0;
int countback = 0;
int countnoneuser = 0;
int countMeanDif = 0;
int countMeanDif2 = 0;

//counts of the second filter, in order to changethe obstacle's position
int count_newuser_left= 0;
int count_newuser_right= 0;
int count_newuser_dif= 0;

int count_int = 0;

String new_data =" ";

float time1=0;
float time2=0;
float time_inter;
int number_actions = 0;

//proximity detection of the obstacle, it is used one of this struct per sensor of the front side 
enum obstacleCloseneses {
  veryCloseOb,
  closeOb,
  farOb
};
obstacleCloseneses right_obstacle = farOb;
obstacleCloseneses left_obstacle = farOb;
obstacleCloseneses front_obstacle = farOb;

//after the filter, set where is the obtacle in the surrounding of the robot
enum obstacle {
  left,
  right,
  front,
  none
};
obstacle last_obstacle = none;      //variable to define the last sonar that saw an obstacle
obstacle actual_obstacle = none;    //variable to define the sonar that is seeing the closest thing
obstacle old_user = none;
obstacle previous_old_user = none;

//Pins sensors
const int trigPin = 32;
const int echoPin = 30;

const int trigPin1 = 28;
const int echoPin1 = 26;

const int trigPin2 = 38;
const int echoPin2 = 36;

const int trigPin3 = 44;
const int echoPin3 = 42;

int count_ent = 0;
NewPing sonar[SONAR_NUM] = { // Sensor object array.
  NewPing(trigPin2, echoPin2, MAX_DISTANCE),  // front sonar
  NewPing(trigPin1, echoPin1, MAX_DISTANCE),  //right sonar
  NewPing(trigPin, echoPin, MAX_DISTANCE),  //left sonar
  NewPing(trigPin3, echoPin3, MAX_DISTANCE),  //back sonar
};

float f_front = MAX_DISTANCE;                //measured distances
float f_right = MAX_DISTANCE;
float f_left = MAX_DISTANCE;
float f_back = MAX_DISTANCE;

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
String action = "";


void setup() {

  old_user = none;
  sonarSetup();//set up the sonars
  virhas.setKpid(2.0, 0.6, 0.5); //high speed PID
  virhas.stop2(); //just in case stop the wheels
  StartTime = millis();

}

void loop() {
  //take information of the sonars. After 300 iterations the meand of the data saved is calculated.
  //Once the position of the object is obtained, the data will be sent to the Raspberry and it will be waitied 
  //for incoming data.
  //With this information decide_movement(String action) is called and the desired action is performed.
  sonarLoop();
}
