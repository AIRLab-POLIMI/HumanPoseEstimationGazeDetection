#include <NewPing.h>
#include <ViRHas.h>
#include <CytronMotorDriver.h>
#include <Encoder.h>


// SONARS

#define SONAR_NUM 4 
#define MAX_DISTANCE 300 // Max distance returned
#define PING_INTERVAL 33 // ms between pings from each sensor
#define BUFF_LEN 5 // Length of the buffer to clean the data

unsigned long pingTimer[SONAR_NUM];
unsigned int cm[SONAR_NUM]; //To store raw ping distances
uint8_t currentSensor = 0; // Which sensor is active

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

float rawFront[BUFF_LEN];
float rawRight[BUFF_LEN];
float rawLeft[BUFF_LEN];
float rawBack[BUFF_LEN];

bool searchingDist[SONAR_NUM] = {true, true, true, true}; // Auxiliary variable to store distaces

float dist[SONAR_NUM] = {MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE}; // Final distance communicated {F, R, L, B}

String positionSonar = "";

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
String movement = "";

int count = 0;

void setup() {
  
  for(uint8_t i = 0; i < BUFF_LEN; i++){
    rawFront[i] = MAX_DISTANCE;
    rawRight[i] = MAX_DISTANCE;
    rawLeft[i] = MAX_DISTANCE;
    rawBack[i] = MAX_DISTANCE;
    }
  
  virhas.setKpid(2.0, 0.6, 0.5);
  virhas.stop();
  sonarSetup();
  
}

void loop() {
  
  // Reading data from SONAR
  
  for(uint8_t i = 0; i < SONAR_NUM; i++){
    if(millis() >= pingTimer[i]){
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      sonar[currentSensor].ping_timer(echoCheck);  // I have cm[i] filled with information at this point
      }
    }
    
  for(uint8_t i = BUFF_LEN -1 ; i < 0; i--){    // I create the circular buffer
    rawFront[i] = rawFront[i-1];
    rawRight[i] = rawRight[i-1];
    rawLeft[i] = rawLeft[i-1];
    rawBack[i] = rawBack[i-1];    
    }
    
  rawFront[0] = cm[0];
  rawRight[0] = cm[1];
  rawLeft[0] = cm[2];
  rawBack[0] = cm[3];
  
  for(uint8_t i = 0; i < SONAR_NUM; i++) searchingDist[i] = true;
  
  for(uint8_t i = 0; i < BUFF_LEN; i++){ // I assign as a distance the last read != from MAX DISTANCE searching dist would remain true if all the vector raw is MAX_DISTANCE
    if(rawFront[i] != MAX_DISTANCE && searchingDist[0]){
      dist[0] = rawFront[i];
      searchingDist[0] = false;  
      }
    if(rawRight[i] != MAX_DISTANCE && searchingDist[1]){
      dist[1] = rawRight[i];
      searchingDist[1] = false;
      }
    if(rawLeft[i] != MAX_DISTANCE && searchingDist[2]){
      dist[2] = rawLeft[i];
      searchingDist[2] = false;
      }
    if(rawBack[i] != MAX_DISTANCE && searchingDist[3]){
      dist[3] = rawBack[i];
      searchingDist[3] = false;
      }
    }
  
  for(uint8_t i = 0; i < SONAR_NUM; i++){
    if(searchingDist[i]){
      dist[i] = MAX_DISTANCE;
      }
    }
    
  
  // Management of serial communication
  
  if(Serial.available() > 0){    
    String data = Serial.readStringUntil('\n');
    if(data == "ready"){  // Regular stream of data, looping inside the python script regularly (python script sends "ready" each while True)
      Serial.print("Stream of Info-> Right: " + String(dist[1]) + " Front: " + String(dist[0]) + " Left: " + String(dist[2]));
      }      
    else{ //I input a new action, data is now the movement
      Serial.print("New action send, executing movement : " + data );
    }

  };
}

void sonarSetup() {
  
  Serial.begin(115200);
  pingTimer[0] = millis() + 75;
  for(uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i-1] + PING_INTERVAL;
  
}
  
void echoCheck() {
  if(sonar[currentSensor].check_timer()){
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
  }
  else{
    cm[currentSensor] = MAX_DISTANCE;  //Distance detected in case no ping is done
    }
}
