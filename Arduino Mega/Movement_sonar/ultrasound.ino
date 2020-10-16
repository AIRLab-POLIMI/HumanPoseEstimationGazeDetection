/*****************************************************************************************************************
 * Description: This code manages the data incoming from the different sensors and by the use of different       *                                                                                         
 * filters it is detected the positionof the object respect to the robot. Right, left, front.                    *
 * In this code it is also creatd the serial connection with the Raspberry Pi in order to excahnge information   *
 * ***************************************************************************************************************
 * Author: Ane San Martin Igarza                                                                                 *
 * ***************************************************************************************************************
 * Last Update: 23/06/2020                                                                                       *
 * **************************************************************************************************************/
void sonarSetup() {// this one is just 
  Serial.begin(115200);
  pingTimer[0] = millis() + 75; // First ping start in 75 ms in order to provide time to the system to start
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL; // The interval to perform the ping from a uktrasound sensor 
                                                    //to the other is 33ms, in order to avoid crossover sensors
}

//function called in each interaction.
//reads information from sensors
void sonarLoop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1)
        //oneSensorCycle(); // Do something with results.
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = MAX_DISTANCE; // distance detected from each sensor in case no ping is done
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
  check_obstacle();
}

void echoCheck() { // If ping echo, set distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Do something with the results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
  Serial.println();
}
// This function analyses the data incomming from the sensors and it analyses , by the distance from each sensor
//where is the object in each iteration. Then, after 300 iteration, this data is processed in order to have a 
//more accurate calculation, information of the real position of the object
void check_obstacle()
{
  // count to perform the mean and therefore the filter of the data
  count ++;
  
  // the distance read from each of the sensors
  f_front = cm[0];
  f_right = cm[1];
  f_left = cm[2];
   if(cm[3]<35){
      in_back = true;
      f_back = cm[3];
    }
  if ( in_back == true && cm[3]>35){
    countback ++;
  }
  else{
    countback = 0;
    }

  if(countback > 500){
    countback = 0;
    in_back = false;
    f_back = cm[3];
    }
  
  //Serial.println(String(f_back));
  
  // when frim the three sensors the distance of the obstacle is far, it is set as no obtacle and far
  if (f_front > FAR_DISTANCE && f_right > FAR_DISTANCE && f_left > FAR_DISTANCE) {
    //sall the counts used in this cose are in order to perform the filtering
    actual_distance = 400;
    if(actual_distance>f_front)
    actual_distance=f_front;
    if(actual_distance>f_right)
    actual_distance=f_right;
    if(actual_distance>f_left)
    actual_distance=f_left;
    if(actual_distance>f_left)
    actual_distance=f_left;
//    if(actual_distance>f_back)
//    actual_distance=f_back;
    no_obstacle = true;
    targetPos = 0;
    actual_obstacle = none;
    countnone = countnone+1;
    distance_far = distance_far + actual_distance; // to obtain the mean distance to the 

  } 
  else {
    // If there is an object detect from which side it comes
    double error=3.0f;
    if(f_back<40){
      back_obstacle = true;
      }
    else{
      back_obstacle = false;
      }
    
    if ((f_front <= f_left && f_front <= f_right) || (f_front <= (f_right + error) && f_front >= (f_right - error) && f_front <= (f_left + error) && f_front >= f_left - error) ) {
      actual_distance = f_front;
      distance_front = distance_front + actual_distance;
      countfront ++;

    } 
    else {
      if ( (f_right < f_front && f_right < f_left)) {
        actual_distance = f_right;
        distance_right = distance_right + actual_distance;
        countright ++;
        
      }
      else{
        if ( (f_left < f_front && f_left < f_right)) {
          actual_distance = f_left;
          distance_left = distance_left + actual_distance;
          countleft++;
          
        }
      }
    }
  }

  // after 300 iteration, the data obtain in each time instant is procesed
 if (count >= MEAN_COUNT)
 {// each 1000 times it is anylyzed by the mean where it comes from the object
  mean_function();
  new_user_function();
  count = 0;
  countnone = 0;
  countfront = 0;
  countright = 0;
  countleft = 0;
  distance_far = 0;
  distance_front = 0;
  distance_right = 0;
  distance_left = 0;
  }

}

void mean_function(){ // filter for the sensor data, it is not recomended to use the whole row data, 
//it is better to recolect some data and analyze from where the user is coming

   last_obstacle = actual_obstacle; // to maintain the information of the previous time
   previous_distance = actual_distance_mean;
   if ((countfront >= countright) && (countfront>= countleft) && ( countfront>= countnone)){
      actual_obstacle = front;
      actual_distance_mean= distance_front/count;
       if (actual_distance_mean > FAR_DISTANCE) 
       front_obstacle = farOb;
       else if(actual_distance_mean <= FAR_DISTANCE && actual_distance_mean > VERYCLOSE_DISTANCE)
       front_obstacle = closeOb;
       else if(actual_distance_mean <= VERYCLOSE_DISTANCE)
       front_obstacle = veryCloseOb;
      
    } else{
        if((countleft >= countright) && (countleft>= countfront) && ( countleft>= countnone)){
          actual_obstacle = left;
          actual_distance_mean= distance_left/count;
            if (actual_distance_mean > FAR_DISTANCE) 
            left_obstacle = farOb;
            else if(actual_distance_mean <= FAR_DISTANCE && actual_distance_mean > VERYCLOSE_DISTANCE)
            left_obstacle = closeOb;
            else if(actual_distance_mean <= VERYCLOSE_DISTANCE)
            left_obstacle = veryCloseOb;
          
        } else{
            if((countright >= countleft) && (countright>= countfront) && ( countright>= countnone)){
              actual_obstacle = right;
              actual_distance_mean = distance_right/count;
                if (f_right > FAR_DISTANCE) 
                right_obstacle = farOb;
                else if(actual_distance_mean  <= FAR_DISTANCE && f_right > VERYCLOSE_DISTANCE)
                right_obstacle = closeOb;
                else if(actual_distance_mean <= VERYCLOSE_DISTANCE)
                right_obstacle = veryCloseOb;
               
            }
            else{
              actual_obstacle = none;
              actual_distance_mean = distance_far/count;
              right_obstacle = farOb;
              front_obstacle = farOb;
              left_obstacle = farOb; 
            }
         }
      }
      
      if (abs( ((actual_distance_mean + previous_distance)/2) - actual_distance_mean) < THRESHOLD_MEAN)
      {
        actual_distance_mean = (actual_distance_mean + previous_distance)/2;
        countMeanDif =0;
        countMeanDif2=0;
        
       }
       else{
        if(actual_distance_mean>FAR_DISTANCE){
          //Serial.println("count" + String(countMeanDif));
          countMeanDif =countMeanDif+1;
          if(countMeanDif>15){
          countMeanDif =0;
          actual_distance_mean = actual_distance_mean;
          }
          else{
          actual_distance_mean =  previous_distance;
          }
        }
        else{
          countMeanDif2 =countMeanDif2+1;
          if(countMeanDif2>5){
          countMeanDif2 =0;
          actual_distance_mean = actual_distance_mean;
          }
          else{
          actual_distance_mean =  previous_distance;
          }
          }
        }
  }

//In some situations it appears some erroneus data, and if it is use directly the meand data may it cahnges
//the objects positions when it should not.
// In order to avoid this situation, it must appear the change this big change (in terms of distance,
//or chnaging from left to right or from right to left) more than a time
void new_user_function(){
   dist_dif = abs(actual_distance_mean-old_dist);
   old_dist = actual_distance_mean;
   previous_movement_direction = movement_direction;
   previous_old_user = old_user;
   //solo importaria este
   if (dist_dif > 150){ //analyze if there is a new user by the distance (threshold 1,5 m)
    count_newuser_dif = count_newuser_dif + 1;
    }
   
   if ((old_user == none) && (( actual_obstacle == right) || (actual_obstacle == left) || ( actual_obstacle== front) || ( actual_obstacle == none))){
      
      old_user = actual_obstacle;
      send_data = true;
      new_data = "new ";
      countnoneuser =0;
      //countnone2=0;
   }
  else{
    if((old_user == right) && ( actual_obstacle == left)){
    count_newuser_left= count_newuser_left + 1;
    countnoneuser = 0;
    }
    else{
      if((old_user == left) and (actual_obstacle == right)){
        count_newuser_right = count_newuser_right + 1;
        countnoneuser = 0;
        }
        else{
          if (((old_user == front) && ((actual_obstacle == right) || (actual_obstacle == left))) || (((old_user == left) || (old_user == right)) && (actual_obstacle == front))){
              old_user = actual_obstacle;
              count_newuser_left= 0;
              count_newuser_right= 0;
              count_newuser_dif= 0;
              countnoneuser = 0;
          }
          else{
            if((((old_user == right) || (old_user == left) || (old_user == front)) && (actual_obstacle == none))){
              countnoneuser = countnoneuser+1;
              if(countnoneuser > 10)
              {
                old_user = actual_obstacle;
                count_newuser_left= 0;
                count_newuser_right= 0;
                count_newuser_dif= 0;
                countnoneuser = 0;
                }
              }
            
            else{
                countnoneuser = 0;
                count_newuser_left= 0;
                count_newuser_right= 0;
                count_newuser_dif= 0;
              }
          }
        }
     }
  }
  // if it appears more than 4 times the object in the right when it was in the left or in the left when it was in the right, it means that is true that a new object has appear.
  // It also happens when the difference of distance is very big from a time instant to the next one
  if ((count_newuser_right>20) || (count_newuser_left>20) || (count_newuser_dif>=1))
  {
        count_newuser_left= 0;
        count_newuser_right= 0;
        count_newuser_dif= 0;
        old_user = actual_obstacle;
        send_data = true;
        new_data = "new ";
        
  }
  else{
    if(send_data == false){
        new_data = "old ";
        old_user = old_user;
    }
  }

    // for doing this one it should be like a kind of trigger because backwards and fordward will not be the whole time it should be like a trigger
    if(action_performing == false){
      if(old_user == none){
          movement_direction = "gone";
          send_data_mov = false;
          
      }
      else{
        if((previous_distance - actual_distance_mean)>=7.0 && send_data_mov == false){
          movement_direction = "forward";
          send_data_mov = true;
          //delay(10000);
          
          }
          else{
            if((previous_distance- actual_distance_mean) <=-7.0 && send_data_mov == false){
               movement_direction = "backward";
               send_data_mov = true;
               //delay(10000);
            }
              else{
                if(abs(previous_distance - actual_distance_mean) < 7.0 && send_data_mov == false){
                  movement_direction = "stopped";
                  send_data_mov = false;
                  }
                }
              }
          }  
    }
    else{
      movement_direction = "stopped";
      }
  //Serial.println("old_dist: "+String(previous_distance)+"old_dist: "+String(actual_distance_mean));
  Serial_connection(new_data + String(old_user)+" "+ String(actual_distance_mean)+" "+String(movement_direction),action_performing);

}
//create a comunication between the Arduino and the raspberry and analyze the incoming data
void Serial_connection(String movement, bool action_performing ) {
    //Serial.println(movement)
    if (Serial.available() > 0) {//when  there is available data to read
      
      String data = Serial.readStringUntil('\n');
      if(data == "ready"){//then, data must be sent
        //when a movement is being performed, it is not desired to send any data until the movement is finished
        if(action_performing == true){
          Serial.print("busy");
        }
        else{
        Serial.print(movement);
        send_data = false;
        send_data_mov = false;
        }
      }
      else{//if it is different from ready then, it is information about the action
        decide_movement(data);
        Serial.print("F");
        }
    }
  }
