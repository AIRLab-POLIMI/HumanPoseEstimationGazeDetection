import time
import serial
import sys

from pynput import keyboard

def on_press(key):
    global movement
    global receiveMovement
    try:
        print("{0} Pressed".format(key.char))        
        if key.char == ("a"):
            movement = "excited_attract"
            receiveMovement = True
        elif key.char == ("s"):
            movement = "interested_excited"
            receiveMovement = True
        elif key.char == ("d"):
            movement = "happy"
            receiveMovement = True
        elif key.char == ("f"):
            movement = "scared"
            receiveMovement = True
        elif key.char == ("g"):
            movement = "very_scared"
            receiveMovement = True
        elif key.char == ("h"):
            movement = "angry"
            receiveMovement = True
        elif key.char == ("j"):
            movement = "move"
            receiveMovement = True
        elif key.char == ("n"):
            movement = "rotateLeft"
            receiveMovement = True
        elif key.char == ("m"):
            movement = "rotateRight"
            receiveMovement = True
        else:
            movement = movement
    except AttributeError:
        movement = movement
        print("Special Key {0} pressed".format(key))

def on_release(key):
    global movement
    
    if key == keyboard.Key.esc:
        movement = "QUIT"          
    print("Movement:" + movement)


ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
ser.flush()

listener = keyboard.Listener(on_press = on_press, on_release = on_release)
listener.start()

movement = "none"
receiveMovement = False

def main():

    time.sleep(2)
    
    global receiveMovement
    global movement
    
    while True:
        
        line = ser.readline().decode('utf-8').rstrip()
        if line: 
            print(line)
        
        if receiveMovement:
                        
            ser.write(bytes((movement+'\n'), encoding='utf-8'))
            receiveMovement = False
        
        if movement == "QUIT":
            break
    


if __name__ == "__main__":
    main()
