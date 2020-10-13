# This scripts is the simplicated part, where it is just analyzed the data obtained and see if this data indicates if the user is new or not

# Import packages
import time
import serial

# in order to stablish the communication between arduino and raspberry
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
ser1 = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
ser.flush()
ser1.flush()

# Global variables for the function
count_newuser_left= 0
count_newuser_right= 0
count_newuser_dif= 0

old_dist = 0
new_dist = 0
distance_mean = 0.0

previous_old_user = "none"
old_user = "none"
new_user = "none"
user_movement = "none"
dist_string = " "
state_system = " "

start_dist = False
start_movement = False
pos_obtained = False
is_new_user = False


def new_user_function():
    # to don't have any lose of information, global variables are needed then,
    # GLOBAL
    global ser 

    # Global variables for the function
    global count_newuser_left
    global count_newuser_right
    global count_newuser_dif

    global old_dist 
    global new_dist
    global distance_mean

    global previous_old_user 
    global old_user 
    global new_user
    global user_movement
    global dist_string 

    global start_dist
    global start_movement
    global pos_obtained
    global is_new_user
    global state_system
    
    
    # Start of the program
    old_dist = new_dist
    
    print("\n")
    ser.write(b"ready\n")
    line = ser.readline().decode('utf-8').rstrip()
    print(line)
    if (line == "busy" or line == "busyF" ):
        user=[]
        position=[]
        distance = []
        movement = []
        state_system = line
        user_obtained = False
        start_position = False
        start_distance = False
        start_movement = False
        is_new_user = False
        #print("line: "+line)
    else:
        user=[]
        position=[]
        distance = []
        movement = []
        state_system = " "
        user_obtained = False
        start_position = False
        start_distance = False
        start_movement = False
        is_new_user = False
        previous_old_user = old_user
        #print("line: "+line)
        for letter in line:  
            #print (letter)
            if (ord(letter) >= 32):
                if ((user_obtained == False) and (ord(letter) == 32) ):
                    user_obtained= True
                    start_position = True
                elif((user_obtained == False) and (ord(letter) !=32)and (ord(letter) != 70)):
                    user.append(letter)
                elif ((start_position==True)  and (ord(letter) !=32)):
                    position.append(letter)
                elif((start_position==True) and (ord(letter) == 32)):
                    start_position = False
                    start_distance = True
                elif((start_distance ==True) and (ord(letter) !=32)):
                    distance.append(letter)
                elif((start_distance ==True) and (ord(letter) == 32)):
                    start_distance = False
                    start_movement = True
                elif((start_movement ==True) and (ord(letter) !=32)):
                    movement.append(letter)
            
        new_user = ''.join(user)
        pos_tot = ''.join(position)
        dist_string = ''.join(distance)
        user_movement = ''.join(movement)
        #print("user: "+new_user)
        #print("pos: " +pos_tot)
        #print("dist: "+ dist_string)
        new_dist = float(dist_string)
        print(user_movement)
        #print(pos_tot)
        
        #print("is new user? " + new_user)
        if (new_user == "new"):
            is_new_user = True
        
        if (pos_tot == "0"):
            old_user = "left"
            #print("left")
        elif(pos_tot == "1"):
            old_user = "right"
            #print("right")
        elif(pos_tot == "2"):
            old_user = "front"
            #print("front")
        else:
            old_user = "none"
            print("none")
        print("user pos: " + old_user)
                
    
    
    
    
            
        

        
    
        
        
    

