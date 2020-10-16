/*****************************************************************************************************************
 * Description: This code decides the movement to be performed depending on the action received by the Raspberry *                                   
 * Pi and then, this one is in charge of calling the correspondent virhas functions in order to control the      *
 * the desired movement                                                                                          *
 * ***************************************************************************************************************
 * Author: Ane San Martin Igarza                                                                                 *
 * ***************************************************************************************************************
 * Last Update: 23/06/2020                                                                                       *
 * **************************************************************************************************************/
//small movements side to side
void funny_lateral(){
  ElapsedTime1 = 0;
  ElapsedTime2 = 0;
  StartTime = millis();
  while(ElapsedTime1<3000)//time for the end of the movement
 {  
      action_performing = true;
      sonarLoop();
      CurrentTime = millis();
      ElapsedTime1 = ElapsedTime1 + (CurrentTime-StartTime);
      ElapsedTime2 = ElapsedTime2 + (CurrentTime-StartTime);
      StartTime = CurrentTime;
      //each 200 ms it will change the direction in order to obtain small movement side to side
      if(ElapsedTime2>200){
      i = i *(-1);
      ElapsedTime2 = 0;
      }
      motion.strafe = 0.8*i;//lateral movement to %80 speed.The i specifies the movement direction (+or-)
      motion.forward = 0;
      motion.angular = 0;
      //this next function is the inverse kinematic, in order to obtain the speed of each wheel
      virhas.run2(
       motion.strafe*_MAX_SPEED,
       motion.forward*_MAX_SPEED,
       motion.angular*_MAX_ANGULAR
      );
      virhas.PIDLoop();//control of the movement
  }
  action_performing = false;
  virhas.stop2();//stop the robot 
  delay(20);
}
//small rotation side to side
void funny_rotate(){
  ElapsedTime1 = 0;
  ElapsedTime2 = 0;
  StartTime = millis();
  while(ElapsedTime1<3000)//time for the end of the movement
 {
    action_performing = true;
    sonarLoop();
    CurrentTime = millis();
    ElapsedTime1 = ElapsedTime1 + (CurrentTime-StartTime);
    ElapsedTime2 = ElapsedTime2 + (CurrentTime-StartTime);
    StartTime = CurrentTime;
    //each 300 ms the direction of the rotation is changed in order to obtain the small side to side rotational
    //movement
    if(ElapsedTime2>300){
    i = i *(-1);
    ElapsedTime2 = 0;
    }
  
    motion.strafe = 0;
    motion.forward = 0;
    motion.angular = 0.8*i;//rotational movement to 80% of the speed. The i specifies the movement direction (+or-)
    //this next function is the inverse kinematic, in order to obtain the speed of each wheel
    virhas.run2(
     motion.strafe*_MAX_SPEED,
     motion.forward*_MAX_SPEED,
     motion.angular*_MAX_ANGULAR
    );
    virhas.PIDLoop();//control of the movement
  }
  action_performing = false;
  virhas.stop2();//stop the robot 
  delay(20);
}
//this is a secind version of the previous movement. In this case it is not only small rotational movements but,
//it also makes a complete rotation after each two small ones
void funny_rotate2(){
  ElapsedTime1 = 0;
  ElapsedTime2 = 0;
  StartTime = millis();
  int count = 0;
  int time_elap = 300;
  while(ElapsedTime1<5600)//time for the end of the movement
 {
    action_performing = true;
    sonarLoop();
    CurrentTime = millis();
    ElapsedTime1 = ElapsedTime1 + (CurrentTime-StartTime);
    ElapsedTime2 = ElapsedTime2 + (CurrentTime-StartTime);
    StartTime = CurrentTime;
    if(ElapsedTime2>time_elap){
    i = i *(-1);
    ElapsedTime2 = 0;
    count = count +1;
    if (time_elap == 2000 ){
      time_elap = 300;//change from big to small rotations
      count = 0;
      }
    }
    if (count >=2){
      time_elap = 2000;
    }
    motion.strafe = 0;
    motion.forward = 0;
    motion.angular = 0.5*i;//in this case the movement is to %50 in angular way
    //this next function is the inverse kinematic, in order to obtain the speed of each wheel
    virhas.run2(
     motion.strafe*_MAX_SPEED,
     motion.forward*_MAX_SPEED,
     motion.angular*_MAX_ANGULAR
    );
    virhas.PIDLoop();//control of the movement
  }
  action_performing = false;
  virhas.stop2();//stop the robot
  delay(20);
}
//In order to express scare, the robot will move slow backwards for a small period of time
void Scared(){
  
  ElapsedTime1 = 0;
  ElapsedTime2 = 0;
  StartTime = millis();
  while(ElapsedTime1<2000)//time for the end of the movement
 {  
    action_performing = true;
    sonarLoop();
    if( back_obstacle == false){
    CurrentTime = millis();
    ElapsedTime1 = ElapsedTime1 + (CurrentTime-StartTime);
    StartTime = CurrentTime;
    motion.strafe = 0;
    motion.forward = -0.3;//negative in order to move the backwards and to 30% of speed
    motion.angular = 0;
    //this next function is the inverse kinematic, in order to obtain the speed of each wheel
    virhas.run2(
     motion.strafe*_MAX_SPEED,
     motion.forward*_MAX_SPEED,
     motion.angular*_MAX_ANGULAR
    );
    virhas.PIDLoop();//control of the movement
  }
  else{
   ElapsedTime1 = 2000;
   virhas.stop2(); //stop the robot
  }
 }
 action_performing = false;
 virhas.stop2();//stop the robot
 delay(20);
}
//In order to express that the robot is veryscare, the robot will move fast backwards for a small period of time
void Very_scared(){
  ElapsedTime1 = 0;
  ElapsedTime2 = 0;
  StartTime = millis();
  while(ElapsedTime1<2000)//time for the end of the movement
 {
    action_performing = true;
    sonarLoop();
    if(back_obstacle == false){
    CurrentTime = millis();
    ElapsedTime1 = ElapsedTime1 + (CurrentTime-StartTime);
    StartTime = CurrentTime;
    motion.strafe = 0;
    motion.forward = -0.5;//negative in order to move the backwards and to 50% of speed
    motion.angular = 0;
    //this next function is the inverse kinematic, in order to obtain the speed of each wheel
    virhas.run2(
     motion.strafe*_MAX_SPEED,
     motion.forward*_MAX_SPEED,
     motion.angular*_MAX_ANGULAR
    );
    virhas.PIDLoop();//control of the movement
  }
  else{
   ElapsedTime1 = 2000;
   virhas.stop2(); //stop the robot

  } 
}
  action_performing = false;
  virhas.stop2();//stop the robot
  delay(20);
}
//Forward movement
void Fordward(int time_for){
  ElapsedTime1 = 0;
  ElapsedTime2 = 0;
  StartTime = millis();
  while(ElapsedTime1<time_for)//time for the end of the movement
 {  
    action_performing = true;
    sonarLoop();
    if(actual_distance_mean>30){
    //Serial.println(actual_distance_mean);
    CurrentTime = millis();
    ElapsedTime1 = ElapsedTime1 + (CurrentTime-StartTime);
    StartTime = CurrentTime;
    motion.strafe = 0;
    motion.forward = 0.3;//30% speed, movement forward
    motion.angular = 0;
    virhas.run2(
     motion.strafe*_MAX_SPEED,
     motion.forward*_MAX_SPEED,
     motion.angular*_MAX_ANGULAR
    );
    virhas.PIDLoop();//control of the movement
    }
    else{
      ElapsedTime1 = time_for;
      //Serial.println("out");
      virhas.stop2();//stop the robot
      //delay(500);
      }
  }
  action_performing = false;
  virhas.stop2();//stop the robot
  delay(20);
}
//BAckwards movement
void Backwards(){
  ElapsedTime1 = 0;
  ElapsedTime2 = 0;
  StartTime = millis();
  while(ElapsedTime1<4000)//time for the end of the movement
 {
    action_performing = true;
    sonarLoop();
    if( back_obstacle == false){
    CurrentTime = millis();
    ElapsedTime1 = ElapsedTime1 + (CurrentTime-StartTime);
    StartTime = CurrentTime;
    motion.strafe = 0;
    motion.forward = -0.3;//30% speed, movement backwards
    motion.angular = 0;
    //this next function is the inverse kinematic, in order to obtain the speed of each wheel
    virhas.run2(
     motion.strafe*_MAX_SPEED,
     motion.forward*_MAX_SPEED,
     motion.angular*_MAX_ANGULAR
    );
    virhas.PIDLoop();//control of the movement
    }
    else{
      ElapsedTime1 = 4000;
      virhas.stop2();//stop the robot 
      //delay(500);
      }
  }
  action_performing = false;
  virhas.stop2();//stop the robot 
  delay(20);
 }
//lateral movement to the left
void lateralLeft(int time_lat){
  ElapsedTime1 = 0;
  ElapsedTime2 = 0;
  StartTime = millis();
  while(ElapsedTime1<time_lat)//time for the end of the movement
 {
    action_performing = true;
    sonarLoop();
    CurrentTime = millis();
    ElapsedTime1 = ElapsedTime1 + (CurrentTime-StartTime);
    StartTime = CurrentTime;
    motion.strafe = 0.3;//30% speed, movement lateral in the left direction
    motion.forward = 0;
    motion.angular = 0;
    //this next function is the inverse kinematic, in order to obtain the speed of each wheel
    virhas.run2(
     motion.strafe*_MAX_SPEED,
     motion.forward*_MAX_SPEED,
     motion.angular*_MAX_ANGULAR
    );
    virhas.PIDLoop();//control of the movement
  }
  action_performing = false;
  virhas.stop2();//stop the robot 
  delay(20);
 }
 //lateral movement to the right
void lateralRight(int time_lat){
 ElapsedTime1 = 0;
  ElapsedTime2 = 0;
  StartTime = millis();
  while(ElapsedTime1<time_lat)//time for the end of the movement
 {
    action_performing = true;
    sonarLoop();
    CurrentTime = millis();
    ElapsedTime1 = ElapsedTime1 + (CurrentTime-StartTime);
    StartTime = CurrentTime;
    motion.strafe = -0.3;//30% speed, movement lateral in the right direction
    motion.forward = 0 ;
    motion.angular = 0;
    //this next function is the inverse kinematic, in order to obtain the speed of each wheel
    virhas.run2(
     motion.strafe*_MAX_SPEED,
     motion.forward*_MAX_SPEED,
     motion.angular*_MAX_ANGULAR
    );
    virhas.PIDLoop();//control of the movement
  }
  action_performing = false;
  virhas.stop2();//stop the robot
  delay(20);
 }
 //rotational movement
void rotate(int time_rot, int sign){
  ElapsedTime1 = 0;
  ElapsedTime2 = 0;
  StartTime = millis();
  while(ElapsedTime1<time_rot)//time for the end of the movement
 {
    action_performing = true;
    sonarLoop();
    CurrentTime = millis();
    ElapsedTime1 = ElapsedTime1 + (CurrentTime-StartTime);
    StartTime = CurrentTime;
    motion.strafe = 0;
    motion.forward = 0;
    motion.angular = sign*0.3;//the sign is used in order to specify the direction of the rotation. To 30% of speed
    //this next function is the inverse kinematic, in order to obtain the speed of each wheel
    virhas.run2(
     motion.strafe*_MAX_SPEED,
     motion.forward*_MAX_SPEED,
     motion.angular*_MAX_ANGULAR
    );
    virhas.PIDLoop();//control of the movement
  } 
  action_performing = false;
  virhas.stop2();//stop the robot
  delay(20);
 }
 //function called when the data is received in order to select the popermovement for each action
 void decide_movement(String action){
    if(action == "init1" || action == "excited_attract")
      funny_lateral();
    if(action ==  "init2" || action == "interested_excited")
      funny_rotate();
    if(action ==  "init3" || action == "happy")
      funny_rotate2();
    if(action == "rotate")
       rotate(1000,1);
    if(action == "rotateRight")
      //strip.clear();
      rotate(300,1);
    if(action == "rotateLeft")
      //strip.clear();
      rotate(300,-1);
    if(action == "lateralRight")
      lateralRight(500);
    if(action ==  "lateralLeft")
      lateralLeft(500);
    if(action ==  "move")
       Fordward(1000);
    if(action ==  "scared")
      Scared();
    if(action == "very_scared")
      Very_scared();
    if (action == "sad")
    if(action == "angry")
    if (action ==  "none" )
      virhas.stop2(); //stop the robot
      delay(500);
    }
