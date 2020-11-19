# This scripts is the simplicated part, where it is just analyzed the data obtained and see if this data indicates if the user is new or not

# Import packages
import time
import serial

# in order to stablish the communication between arduino and raspberry
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
ser1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
ser.flush()
ser1.flush()

# Global variables for the function

new_dist = 0
old_user = "none"
moving = 0

def new_user_function():
    # to don't have any lose of information, global variables are needed then,
    # GLOBAL
    global ser 

    # Global variables for the function
     
    global new_dist
    global old_user  
    global moving
    sending = "ready"
    
    ser.write(bytes((sending+'\n'), encoding='utf-8'))
    line = ser.readline().decode('utf-8').rstrip()
    
    if line:
        print(line)
        data = line.split()
        if len(data) == 3:
            old_user = data[0]
            new_dist = float(data[1])
            moving = data[2]
    readFromMega = False
    
            
        

        
    
        
        
    

