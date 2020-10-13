import serial
from subprocess import run 

previous_action = "none"
next_action = "none"
state_user = ""
current_action = "none"

def human_verification(angle_mean, user, count): #return if the object detected by sonar is a human
    #print("angle: " + str(angle_mean)+", User:" + user)
    if (( user== "front") and ((angle_mean >=-20 ) and (angle_mean <= 20)) and (count >=2)):
        print("Human front")
        tracking_a_user = True
    elif (( user== "right") and (angle_mean >= 7) and (count >=2)):
        print("human right")
        tracking_a_user = True
    elif ((user == "left") and (angle_mean <=-7) and (count >=2)):
        print("Human left")
        tracking_a_user = True
    else:
        tracking_a_user = False
        print("Arduino: object, not human")
    return tracking_a_user

def send_action_arduino(actual_action, reply, ser, tracking_a_user):
    if((actual_action != reply) and (tracking_a_user == True)):
        actual_action = reply
        # then that a new action want to be applied.
        # If there is a user being tracked, send the message tu the arduino to perform the correspondent action
        ser.write(bytes((actual_action+'\n'), encoding='utf-8'))
        line = ser.readline().decode('utf-8').rstrip()
        reproduce_action_sound(actual_action)
        print(line)
    elif(tracking_a_user == True):
        # it means that it is the same message as before, then, no new action will be sent
        actual_action = reply
    else:
        actual_action = " "
          
    return actual_action

def send_uno_lights(ser1,action):
        # actions sent to the Arduino for the initial interaction
        ser1.write(bytes((action+'\n'), encoding='utf-8'))
        # The system will not continue until the movement has been performed
        #reproduce_action_sound(action)
        line1 = ser1.readline().decode('utf-8').rstrip()
        
def send_initial_action_arduino(actual_action, ser,action):
        # actions sent to the Arduino for the initial interaction
        ser.write(bytes((actual_action+'\n'), encoding='utf-8'))
        
        # The system will not continue until the movement has been performed
        reproduce_action_sound(action)
        line = ser.readline().decode('utf-8').rstrip()
        
        #reproduce_action_sound(actual_action)
        print(line)


def reproduce_action_sound(action):
    if(action!="none" and action!="scared" and action!="move_find"):
        if(action == "excited"):
            run("omxplayer --vol 600 /home/pi/tensorflow1/models/research/object_detection/sounds/Excited_R2D2.mp3",shell=True)
        elif(action == "sad"):
            run("omxplayer --vol 600 /home/pi/tensorflow1/models/research/object_detection/sounds/Sad_R2D2.mp3",shell=True)
        elif(action == "out"):
            run("omxplayer --vol 600 /home/pi/tensorflow1/models/research/object_detection/sounds/Playful_R2D2.mp3",shell=True)
        elif(action == "found"):
            run("omxplayer --vol 600 /home/pi/tensorflow1/models/research/object_detection/sounds/founding.mp3",shell=True)
        elif(action == "move"):
            run("omxplayer --vol 600 /home/pi/tensorflow1/models/research/object_detection/sounds/fordward.mp3",shell=True)
        elif(action == "excited_attract"):
            run("omxplayer --vol 600 /home/pi/tensorflow1/models/research/object_detection/sounds/Excited_R2D2.mp3",shell=True)
        elif(action == "interested_excited"):
            run("omxplayer --vol 600 /home/pi/tensorflow1/models/research/object_detection/sounds/founding.mp3",shell=True)
        elif(action == "happy"):
            run("omxplayer --vol 600 /home/pi/tensorflow1/models/research/object_detection/sounds/Laughing_R2D2.mp3",shell=True)
        elif(action == "very_scared"):
            run("omxplayer --vol 600 /home/pi/tensorflow1/models/research/object_detection/sounds/Screaming_R2D2.mp3",shell=True)
        elif(action == "angry"  ):
            run("omxplayer --vol 600 /home/pi/tensorflow1/models/research/object_detection/sounds/Snappy_R2D2.mp3",shell=True)
            
def decide_action(action,movement):
    global previous_action
    global current_action
    global state_user
    state = state_user
    previous_action = current_action
    obtain_user_state(action,movement)
    if(((state =="interested_scared" )and(previous_action == "interested_excited")) or ((state =="interested_scared" )and(previous_action == "happy")) or ((state =="interested_scared" )and(previous_action == "scared")) or ((state =="interested_scared" )and(previous_action == "very_scared")) or ((state =="interested_scared" )and(previous_action == "sad")) or ((state =="scared" )and(previous_action == "interested_excited")) or ((state =="scared" )and(previous_action == "happy")) or ((state =="scared" )and(previous_action == "angry")) or ((state =="scared" )and(previous_action == "sad")) or ((state =="scared" )and(previous_action == "none"))):
        current_action = "excited_attract"
    elif(((state =="interested_scared" )and(previous_action == "excited_attract")) or ((state =="interested_scared" )and(previous_action == "angry")) or ((state =="interested_scared" )and(previous_action == "none")) or ((state =="scared" )and(previous_action == "excited_attract")) or ((state =="scared" )and(previous_action == "scared")) or ((state =="scared" )and(previous_action == "very_scared"))):
        current_action = "interested_excited"
    elif(state == "interested_interacting"):
        current_action = "happy"
    elif(((state =="scared_aggressive" )and(previous_action == "interested_excited")) or ((state =="scared_aggressive" )and(previous_action == "excited_attract")) or ((state =="scared_aggressive" )and(previous_action == "happy")) or ((state =="scared_aggressive" )and(previous_action == "none")) or ((state =="gaming_aggressive" )and(previous_action == "interested_excited")) or ((state =="gaming_aggressive" )and(previous_action == "excited_attract")) or ((state =="gaming_aggressive" )and(previous_action == "happy")) or ((state =="gaming_aggressive" )and(previous_action == "none"))):
        current_action = "scared"
    elif(((state =="scared_aggressive" )and(previous_action == "scared")) or ((state =="gaming_aggressive" )and(previous_action == "scared")) or ((state =="gaming_aggressive" )and(previous_action == "very_scared")) or ((state =="gaming_aggressive" )and(previous_action == "angry"))):
        current_action = "very_scared"
    elif(((state =="scared_aggressive" )and(previous_action == "angry")) or ((state =="scared_aggressive" )and(previous_action == "sad")) or ((state =="gaming_aggressive" )and(previous_action == "sad"))):
        current_action = "sad"
    elif(((state =="scared_aggressive" )and(previous_action == "very_scared"))):
        current_action = "angry"
    
def obtain_user_state(action,movement):
    global state_user
    if(((action == "touch")and(movement == "forward")) or ((action == "hug")and(movement == "forward")) or ((action == "none")and(movement == "forward")) or ((action == "hug")and(movement == "stopped"))):
        state_user = "interested_interacting"
    elif (((action == "push")and(movement == "forward")) or ((action == "hit")and(movement == "forward")) or ((action == "push")and(movement == "stopped")) or ((action == "hit")and(movement == "stopped"))):
        state_user = "scared_aggressive"
    elif(((action == "touch")and(movement == "backward")) or ((action == "touch")and(movement == "stopped")) or ((action == "none")and(movement == "stopped"))):
        state_user = "interested_scared"
    elif(((action == "none")and(movement == "backward"))):
        state_user = "scared"
    elif(((action == "strongHug")and(movement == "forward")) or ((action == "strongHug")and(movement == "stopped"))):
        state_user = "gaming_aggressive"
    else:
        state_user=state_user
    #print(state_user)
        
        
    
    
