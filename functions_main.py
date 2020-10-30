import serial
from subprocess import run 

previous_action = "none"
next_action = "none"
state_user = ""
current_action = "none"

def human_verification(angle_mean, user, count): #return if the object detected by sonar is a human
    #print("angle: " + str(angle_mean)+", User:" + user)
    if (( user== "front") and ((angle_mean >=-10 ) and (angle_mean <= 10)) and (count >=2)):
        print("Human front")
        tracking_a_user = True
    elif (( user== "right") and (angle_mean >= 10) and (count >=2)):
        print("human right")
        tracking_a_user = True
    elif ((user == "left") and (angle_mean <=-10) and (count >=2)):
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
        #reproduce_action_sound(actual_action)
        #print(line)
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
        
def send_initial_action_arduino(actual_action, ser, action):
        # actions sent to the Arduino for the initial interaction
        ser.write(bytes((actual_action+'\n'), encoding='utf-8'))
        reproduce_action_sound(action)



def reproduce_action_sound(action):
    if(action!="none" and action!="move_find"):
        if(action == "excited"):
            run("lxterminal -e omxplayer --vol 600 sounds/Excited_R2D2.mp3 &",shell=True)
        elif(action == "sad"):
            run("lxterminal -e omxplayer --vol 600 sounds/Sad_R2D2.mp3 &",shell=True)
        elif(action == "out"):
            run("lxterminal -e omxplayer --vol 600 sounds/Playful_R2D2.mp3 &",shell=True)
        elif(action == "found"):
            run("lxterminal -e omxplayer --vol 600 sounds/founding.mp3 &",shell=True)
        elif(action == "move"):
            run("lxterminal -e omxplayer --vol 600 sounds/fordward.mp3 &",shell=True)
        elif(action == "excited_attract"):
            run("lxterminal -e omxplayer --vol 600 sounds/Excited_R2D2.mp3 &",shell=True)
        elif(action == "interested_excited"):
            run("lxterminal -e omxplayer --vol 600 sounds/founding.mp3 &",shell=True)
        elif(action == "happy"):
            run("lxterminal -e omxplayer --vol 600 sounds/Laughing_R2D2.mp3 &",shell=True)
        elif(action == "very_scared"):
            run("lxterminal -e omxplayer --vol 600 sounds/Screaming_R2D2.mp3 &",shell=True)
        elif(action == "scared"):
            run("lxterminal -e omxplayer --vol 600 sounds/Screaming_R2D3.mp3 &",shell=True)
        elif(action == "angry"):
            run("lxterminal -e omxplayer --vol 600 sounds/Snappy_R2D2.mp3 &",shell=True)
            
def decide_action(action):
    global previous_action
    global current_action
    global state_user
    state = state_user
    obtain_user_state(action)
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
    previous_action = current_action
    
def obtain_user_state(action):
    global state_user
    if action == "touch": state_user = "interested_scared"
    elif action == "push": state_user = "scared"
    elif action == "hit": state_user = "scared_aggressive"
    elif action == "hug": state_user = "interested_interacting"
    elif action == "strongHug": state_user = "gaming_aggressive"
    else: state_user = state_user        
        
    
    
