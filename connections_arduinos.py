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
        
        data = line.split()
        if len(data) == 4:
            new_user = data[0]
            pos_tot = data[1]
            new_dist = float(data[2])
            user_movement = data[3]
            
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
                    
    
    
    
    
            
        

        
    
        
        
    

