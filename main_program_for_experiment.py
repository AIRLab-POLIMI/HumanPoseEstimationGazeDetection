#!/usr/bin/env python3

import argparse
import sys
import cv2
import time
import numpy as np
import math

from datetime import datetime
from threading import Thread

from edgetpu.basic import edgetpu_utils
from pose_engine import PoseEngine
from finalmodel import prepare_modelSingle
from openvino.inference_engine import IECore
from detector import Detector
from pynput import keyboard

import functions_main
import connections_arduinos as arduino #new_user_fun


import logging
logging.basicConfig(format="[ %(levelname)s ] %(message)s",
                    level=logging.INFO,
                    stream=sys.stdout)
log = logging.getLogger()

EDGES = (
    ('nose', 'left eye'),
    ('nose', 'right eye'),
    ('nose', 'left ear'),
    ('nose', 'right ear'),
    ('left ear', 'left eye'),
    ('right ear', 'right eye'),
    ('left eye', 'right eye'),
    ('left shoulder', 'right shoulder'),
    ('left shoulder', 'left elbow'),
    ('left shoulder', 'left hip'),
    ('right shoulder', 'right elbow'),
    ('right shoulder', 'right hip'),
    ('left elbow', 'left wrist'),
    ('right elbow', 'right wrist'),
    ('left hip', 'right hip'),
    ('left hip', 'left knee'),
    ('right hip', 'right knee'),
    ('left knee', 'left ankle'),
    ('right knee', 'right ankle'),
)

def build_argparser():
    parser = argparse.ArgumentParser()
    parser.add_argument("-m_od", "--model_od", type=str, default= "models/ssdlite_mobilenet_v2/FP16/ssdlite_mobilenet_v2.xml",
                        help="path to model of object detector to be infered in NCS2, in xml format")

    parser.add_argument("-m_hpe", "--model_hpe", default="models/posenet_mobilenet_v1_075_481_641_quant_decoder_edgetpu.tflite", type=str,
                            help="path to model of human pose estimator to be infered in Google Coral TPU, TFlite model. Assigned one by default")

    parser.add_argument("-i", "--input", type=str, nargs='+', default='0', help="path to video or image/images")
    parser.add_argument("-d", "--device", type=str, default='MYRIAD', required=False,
                        help="Specify the target to infer on CPU, GPU, or MYRIAD")
    parser.add_argument("--person_label", type=int, required=False, default=1, help="Label of class person for detector")
    parser.add_argument("--modality", type=str, default="Multi", help="Define the modality of representation of the output. Set single to visualize the skeleton of the main actor")
    parser.add_argument("--no_show", help='Optional. Do not display output.', action='store_true')
    return parser


class WebcamVideoStream:
    def __init__(self, width, height,src=0):
        self.width = width
        self.height = height
        #initialize the video stream and read the first frame
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, self.height)
        (self.grabbed, self.frame) = self.stream.read()
        
        #inizializing the variable used to indicate if the thread shoud be stopped
        self.stopped = False
            
    def start(self):
        #start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self
    
    def update(self):
        #keep looping until the thread is stopped
        while True:
            #if the thread indicator variable is set, stop the thread. Otherwise read the next frame
            if self.stopped: return
            else: (self.grabbed, self.frame) = self.stream.read()
        
    def read(self):
        #return the most recent frame
        return self.frame
    
    def stop(self):
        #stop the thread
        self.stopped = True
        

def on_press(key):
    global child_action
    global JointAttention
    global receiveAction
    try:
        print("{0} Pressed".format(key.char))        
        if key.char == ("a"):
            child_action = "touch"
            JointAttention = False
            receiveAction = True
        elif key.char == ("s"):
            child_action = "push"
            JointAttention = False
            receiveAction = True
        elif key.char == ("d"):
            child_action = "hit"
            JointAttention = False
            receiveAction = True
        elif key.char == ("f"):
            child_action = "hug"
            JointAttention = False
            receiveAction = True
        elif key.char == ("g"):
            child_action = "strongHug"
            JointAttention = False
            receiveAction = True
        elif key.char == ("h"):
            child_action = "none"
            JointAttention = False
            receiveAction = True
        elif key.char == ("j"):
            child_action = "joint"
            JointAttention = False
            receiveAction = True
        else:
            child_action = child_action
    except AttributeError:
        child_action = child_action
        print("Special Key {0} pressed".format(key))

def on_release(key):
    global child_action
    global JointAttention
    if key == keyboard.Key.esc:
        child_action = "QUIT"
        JointAttention = False    
    print("Child action: " + child_action)

def draw_on_img(img, centr, res, uncertainty): #gaze drawing
    res[0] *= img.shape[0]
    res[1] *= img.shape[1]
    angle = -math.degrees(math.atan2(res[1],res[0]))

    norm1 = res / np.linalg.norm(res)
    norm1[0] *= img.shape[0]*0.30
    norm1[1] *= img.shape[0]*0.30
    
    point = centr + norm1 
    
    if centr[0]!=0 and centr[1]!=0:
        result = cv2.arrowedLine(img, (int(centr[0]),int(centr[1])), (int(point[0]),int(point[1])), (0,0,255), thickness=2, tipLength=0.1)
    else:
        result = cv2.circle(img, (0,0), 1, (0,0,0))
    result = cv2.putText(result, " Gaze Uncertainty {:.3f}".format(uncertainty), (10,450), cv2.FONT_HERSHEY_SIMPLEX ,0.5, (0,255,0),1)    
    result = cv2.putText(result, " Gaze Angle {:.3f}".format(angle), (10,470), cv2.FONT_HERSHEY_SIMPLEX ,0.5, (0,255,0),1)
    return result, angle

def elaborate_gaze(img, head, score, model_gaze):
    centroid = compute_centroid(head)
    max_dist = max([dist_2D(j, centroid) for j in head])
    new_repr= np.array(head) - np.array(centroid)
    result= []
    for point in head:
        if point[0] != 0:
            new_repr = np.array(point) - np.array(centroid)
            result.append([new_repr[0]/max_dist, new_repr[1]/max_dist])
        else: result.append([0,0])
        
    features = [item for sublist in result for item in sublist]
    featMap = np.asarray(features)
    confMap = np.asarray(score)
    featMap = np.reshape(featMap,(1,10))
    confMap = np.reshape(confMap,(1,5))
    centr = np.asarray(centroid)
    centr = np.reshape(centr,(1,2))
    poseFeats = np.concatenate((centr, featMap, confMap), axis=1)
    data =[]
    data.append(poseFeats)
    ld = np.array(data)
    ld = np.squeeze(ld,axis=1)
    X_ = np.expand_dims(ld[:, 2:],axis=2)
    pred_ = model_gaze.predict(X_, batch_size=32, verbose=0)
    gazeDirections = pred_[0,:-1]
    Uncertainties = np.exp(pred_[0,-1])
    Centroids = ld[0,0:2]
    if args.no_show:
        return pred_
    else:
        result, angle = draw_on_img(img, Centroids, gazeDirections, Uncertainties)
        return result, angle, Centroids, pred_
    
def elaborate_pose(result, threshold=0.7):  #the order of the first keypoints is given in pose_engine.py (var list KEYPOINTS)
    i=0
    xys = {}
    score = {}
    for pose in result:
       i+=1
       for label, keypoint in pose.keypoints.items(): 
           if i==1 :     #if the pose is the main one
               if (label == "nose" or label == "left eye" or label == "right eye" \
                    or label == "left ear" or label == "right ear") and keypoint.score > threshold:
                    xys[label] = (int(keypoint.yx[1]), int(keypoint.yx[0]))
                    score[label] = keypoint.score
               else:
                   xys[label] = (0,0)
                   score[label] = 0
    head = np.zeros((5,2)).astype(np.int)
    
    #Head and scores must be ordered like [nose, reye, leye, rear, lear] for the gaze model
    head[0][0] = xys["nose"][0]
    head[0][1] = xys["nose"][1]
    head[1][0] = xys["right eye"][0]
    head[1][1] = xys["right eye"][1]
    head[2][0] = xys["left eye"][0]
    head[2][1] = xys["left eye"][1]
    head[3][0] = xys["right ear"][0]
    head[3][1] = xys["right ear"][1]
    head[4][0] = xys["left ear"][0]
    head[4][1] = xys["left ear"][1]
    scores = np.zeros((5,1))
    scores[0] = score["nose"]
    scores[1] = score["right eye"]
    scores[2] = score["left eye"]    
    scores[3] = score["right ear"]
    scores[4] = score["left ear"]
    ts = time.time()

    return head, scores
        
def draw_pose(img, pose, person, mode, i, threshold=0.7):
    xys = {}
    if mode == "single":
        for label, keypoint in pose.keypoints.items():
            if keypoint.score < threshold: continue
            xys[label] = (int(keypoint.yx[1]), int(keypoint.yx[0]))
            if (keypoint.yx[1]>person[0] and keypoint.yx[1]<(person[0]+person[2])) and (keypoint.yx[0]>person[1] and keypoint.yx[0]<(person[1]+person[3])):
                img = cv2.circle(img, (int(keypoint.yx[1]), int(keypoint.yx[0])), 5, (0, 255, 0), -1)
                cv2.putText(img, label, (int(keypoint.yx[1])+3, int(keypoint.yx[0])-7), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255))
    else:
        for label, keypoint in pose.keypoints.items():
            if keypoint.score < threshold: continue
            xys[label] = (int(keypoint.yx[1]), int(keypoint.yx[0]))
            if i == 1:
                img = cv2.circle(img, (int(keypoint.yx[1]), int(keypoint.yx[0])), 5, (0, 255, 0), -1)
                cv2.putText(img, label+str(i), (int(keypoint.yx[1])+3, int(keypoint.yx[0])-7), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255))
            
        
    for a, b in EDGES:
        if a not in xys or b not in xys: continue
        ax, ay = xys[a]
        bx, by = xys[b]
        if mode == "single":
            if ax>person[0] and ax<(person[0]+person[2]) and ay>person[1] and ay<(person[1]+person[3]):
                img = cv2.line(img, (ax, ay), (bx, by), (255,0,0), 2)
        else:
            img = cv2.line(img, (ax, ay), (bx, by), (255,0,0), 2)

def overlay_on_image(frames, result, model_width, model_height,person,mode):

    color_image = frames

    if isinstance(result, type(None)):
        return color_image
    img_cp = color_image.copy()

    i=0;
    for pose in result:
        i+=1
        draw_pose(img_cp, pose, person, mode, i)

    return img_cp

def compute_centroid(points):

    mean_x = np.mean([p[0] for p in points if p[0]!=0])
    mean_y = np.mean([p[1] for p in points if p[0]!=0])
    
    if math.isnan(mean_x) or math.isnan(mean_x):
        mean_x=0;
        mean_y=0;

    return [mean_x, mean_y]

def dist_2D(p1, p2):

    p1 = np.array(p1)
    p2 = np.array(p2)


    squared_dist = np.sum((p1 - p2)**2, axis=0)
    return np.sqrt(squared_dist)
    
def target_camera_angle(target, im_width):
    xmin = target[0]
    xmax = target[0] + target[2]
    centerx = xmin + ((xmax-xmin)/2)
    angle = (0.09375*centerx)-30
    return angle


child_action = " "
JointAttention = False
receiveAction = False
breakFromKey = False
listener = keyboard.Listener(on_press = on_press, on_release = on_release)
listener.start()

def run_demo(args):
    
    time.sleep(2)
    arduino.new_user_function()
    
    functions_main.send_uno_lights(arduino.ser1,"none") 
    functions_main.send_uno_lights(arduino.ser1, "move")
    
    #Setting communications  

    if args.model_hpe == "models/posenet_mobilenet_v1_075_481_641_quant_decoder_edgetpu.tflite":
        camera_width  = 640
        camera_height = 480
    elif args.model_hpe == "models/posenet_mobilenet_v1_075_353_481_quant_decoder_edgetpu.tflite":
        camera_width  = 480
        camera_height = 360
    else:
        camera_width  = 1280
        camera_height = 720
        
    model_width   = 640
    model_height  = 480
    
    #Loading labels based on the Object Detector model (different labels for different networks)
    if args.model_od == "models/mobilenet-ssd.xml":
        labels = ['Background','Person','Car', 'Bus', 'Bicycle','Motorcycle'] #???
    elif args.model_od == "models/ssdlite_mobilenet_v2/FP16/ssdlite_mobilenet_v2.xml" or "models/ssdlite_mobilenet_v2/FP32/ssdlite_mobilenet_v2.xml" :
        labels = [" ",]
        file1 = open("models/ssdlite_mobilenet_v2/labels.txt", 'r')
        while True:
            line = file1.readline().rstrip().split()
            if len(line) == 1: line = " "
            elif len(line) == 2: line = line[1]
            elif len(line) == 3: line = line[1] + " " + line[2]
            labels.append(line)
            if not line:
                labels.pop()
                break
        file1.close()
            
    # Posenet - Google Coral Inference
    print("#----- Loading Posenet - Coral Inference -----#")
    devices = edgetpu_utils.ListEdgeTpuPaths(edgetpu_utils.EDGE_TPU_STATE_UNASSIGNED)
    engine = PoseEngine(args.model_hpe, devices[0])
    res = []
    print("#-----Done-----#")
    
    #Mobilenet - NCS2 Inference
    print("#-----Loading SSDLite Mobilenet v2 - NCS2 Inference -----#")
    ie = IECore()
    detector_object = Detector(ie, path_to_model_xml=args.model_od, device=args.device, label_class=args.person_label)
    print("#-----Done-----#")
    

    #Custom Gaze Estimator predictor
    print("#-----Loading Gaze Estimator Net -----#")    
    model_gaze = prepare_modelSingle('relu')
    model_gaze.load_weights('/home/pi/Detection-and-Human-Pose-Estimation---RASPBERRY/models/trainedOnGazeFollow_weights.h5')    
    print("#-----Done-----#")
    
    #Framerate variables
    fps = ""
    
    #Human Interaction variables
    TIME_OUT = 40 # How much time do i have if i'm searching a human during the interaction loop?
    TIME_OUT_HUM = 10 # How much time can I stay without human?
    JA_TIME = 30 #duration of JA task analisys
    child_action_prec = "none"
    tracking_a_user = False #is the obstacle i read from sonar an human?
    Finding_human = False #am i looking for a human?
    global receiveAction
    global JointAttention
    global child_action
    firstTime = True
    lookTo = ""
    previousAngle = 0
    angle = 0
    interaction = 0
    tooCloseCount = 0
    tooFarCount = 0
    
    
    #--> Counting the time that manages the reseach of a human
    time_out_system = 0
    start_time_out_system = 0
    current_time_out_system = 0
    
    #--> Counting the time of interaction
    time_out_system_hum = 0
    start_time_out_system_hum = 0
    current_time_out_system_hum = 0
    
    #---> Counting JA task observation
    start_time_JA = 0
    duration_JA = 0
    actual_time_JA = 0
    
    start_time_LOOKING = 0 
    duration_LOOKING = 0
    actual_time_LOOKING = 0
    
    toleranceFrame = 0
    greetedTeddy = False
    oldTargetBox = []
    targetBox = []
    
    #Camera Thread
    vs = WebcamVideoStream(camera_width, camera_height, src=0).start()
    
    functions_main.send_uno_lights(arduino.ser1,"none") 
    
    while True:
        
        t1 = time.perf_counter()
        frame = vs.read()
        frame = cv2.resize(frame, (model_width, model_height))
        color = (0, 0, 255)
                
        arduino.new_user_function()        
   
        # Run Object Detection                
        previousAngle = angle
        
        bboxes, labels_detected, score_detected, bboxes_person, bboxes_teddy = detector_object.detect(frame)        
        main_person = [0,0,0,0]
        main_teddy = [0,0,0,0]
        areas=[]
        for bbox in bboxes_person:
            area = bbox[2]*bbox[3]
            areas.append(area)
        if areas:
            box_person_num = areas.index(max(areas))
            main_person = [bboxes_person[box_person_num][0],bboxes_person[box_person_num][1],bboxes_person[box_person_num][2],bboxes_person[box_person_num][3]]
            angle = target_camera_angle(main_person, camera_width)
        else:
            angle = 0
            
        areas = []
        
        for bbox in bboxes_teddy:
            area = bbox[2]*bbox[3]
            areas.append(area)
        if areas:
            box_teddy_num = areas.index(max(areas))
            main_teddy = [bboxes_teddy[box_teddy_num][0], bboxes_teddy[box_teddy_num][1],bboxes_teddy[box_teddy_num][2],bboxes_teddy[box_teddy_num][3]]
            angleTeddy = target_camera_angle(main_teddy, camera_width)
            targetBox = [[main_teddy[0], main_teddy[1]], [main_teddy[0]+main_teddy[2], main_teddy[1]+main_teddy[3]], [main_teddy[0]+main_teddy[2], main_teddy[1]], [main_teddy[0], main_teddy[1]+main_teddy[3]]]
        
        else:
            angleTeddy = 0
            
        ####-----START HUMAN INTERACTION-----####
        count = 0
        
        #arduino.new_user_function() #Connect with the Mega and obtain data from sensors
                    
        #interaction = 0 or interaction=1 is when the system is trying to estabilish an interaction with the child
        #interaction = 2 is when the robot is already interacting with the human
        
        if JointAttention:
            time_out_system_hum = 0
            print("Joint Attention Task")        
            if angleTeddy != 0 and greetedTeddy == False: #there is a teddy in the FOV of robot
                print("Teddy Bear in the FOV")
                if (abs(angleTeddy)<=10): #the teddy is in front of the robot
                    if arduino.new_dist < 80:
                        print("Inviting to play with the teddy...")
                        functions_main.send_uno_lights(arduino.ser1, "excited_attract")
                        functions_main.send_initial_action_arduino("backForth", arduino.ser, "happy")
                        functions_main.send_initial_action_arduino("scared", arduino.ser, "none")
                        greetedTeddy = True
                        start_time_LOOKING = time.time()
                    elif arduino.new_dist > 80:
                        print("Teddy too far, approaching...")
                        functions_main.send_initial_action_arduino("move", arduino.ser, "none")
                elif angleTeddy >=10:
                        print("Adjusting right...")
                        functions_main.send_uno_lights(arduino.ser1, "rotateRight")
                        functions_main.send_initial_action_arduino("rotateRight", arduino.ser, "none")
                elif angleTeddy <= -10:
                    print("Adjusting left...")
                    functions_main.send_uno_lights(arduino.ser1, "rotateLeft")
                    functions_main.send_initial_action_arduino("rotateLeft", arduino.ser, "none")
            elif targetBox == oldTargetBox: #if there is no Teddy Bear identified for a frame
                toleranceFrame += 1
                if toleranceFrame == 10 and greetedTeddy == False: #if there actually is no teddy bear in the scene
                    print("Searching for a Teddy Bear...")
                    functions_main.send_uno_lights(arduino.ser1, "rotateRight")
                    functions_main.send_initial_action_arduino("rotateRight", arduino.ser, "none")
                    toleranceFrame = 0
                elif toleranceFrame == 20 and greetedTeddy:
                    #Task Completed (?)
                    toleranceFrame = 0
                    
            if greetedTeddy:
                #I pretend that it did not spontaneously went out of the scene, so I can assume that Teddy is still on position
                actual_time_JA = time.time()
                duration_JA = duration_JA + (actual_time_JA - start_time_JA)
                start_time_JA = actual_time_JA
                print("Time JA: {:.1f}".format(duration_JA))
                prep_image = frame[:, :, ::-1].copy() 
                res, inference_time = engine.DetectPosesInImage(prep_image)      
                if res:
                    head, scores_head = elaborate_pose(res)
                    frame = overlay_on_image(frame, res, model_width, model_height, main_person, args.modality)
                    frame, gazeAngle, headCentroid, prediction = elaborate_gaze(frame, head, scores_head, model_gaze)
                    if headCentroid[1] == 0: gazeAngle = 0
                    if gazeAngle < 0: gazeAngle = 360 + gazeAngle
                    targetAngleMax = -360
                    targetAngleMin = 360
                    if targetBox:
                        for vertices in targetBox:
                            targetAngle=-math.degrees(math.atan2(vertices[1]-headCentroid[1],vertices[0]-headCentroid[0]))
                            cv2.line(frame, (vertices[0],vertices[1]), (int(headCentroid[0]), int(headCentroid[1])), (255,255,255), 1)
                            if targetAngle > targetAngleMax:
                                targetAngleMax = targetAngle
                                print("update max")
                            if targetAngle < targetAngleMin:
                                targetAngleMin = targetAngle
                                print("update min")
                        if targetAngleMax < 0: targetAngleMax = 360 + targetAngleMax
                        if targetAngleMin < 0: targetAngleMin = 360 + targetAngleMin
                        print(targetAngleMin, targetAngleMax)
                        print(gazeAngle)
                        actual_time_LOOKING = time.time()
                        if gazeAngle > (targetAngleMin-5) and gazeAngle < (targetAngleMax+5):
                            color = (0,255,0)
                            duration_LOOKING = duration_LOOKING + abs(actual_time_LOOKING - start_time_LOOKING)
                        start_time_LOOKING = actual_time_LOOKING
                print("Time spent looking at the teddy: {:.1f}".format(duration_LOOKING))
        else:
            if duration_JA != 0 and duration_LOOKING != 0:
                now = datetime.now()
                dt_string = now.strftime("%d/%m/%y %H:%M:%S")
                data = dt_string + ',' + str(duration_JA) + ',' + str(duration_LOOKING) + '\n'
                with open('experiment_records.csv','a') as fp:
                    print("Record stored successfully")
                    fp.write(data)   
            duration_JA = 0
            duration_LOOKING = 0
                    
        if interaction != 2 and not JointAttention: #If I'm not interacting with the human
            print("Interaction != 2, I'm not interacting with the human")
            if arduino.old_user != "none": #if an object is detected by the sonar, check if it is a human
                print("Object detected by sonars")                              
                if angle != 0:  # this can be replaced with a check if it is human, so this translate to if there is a human 
                    print("Human detected in the FOV")
                    count = 4
                    tracking_a_user = functions_main.human_verification(angle, arduino.old_user, count) #it check if obstacle from sonar is a human
                    if tracking_a_user == True:
                        print("Object from sonar is a human")
                        interaction = 1
                        ### --- I Should check if the path between the robot and the child is clear
                        ##if the human is free
                        ##if the human is free for few instant
                        interaction = 2
                        start_time_out_system_hum = time.time()
                        #THERE IS A HUMAN READY TO INTERACT WITH!
                        Finding_human = False
                    else: #if it find an object that is not a human, must rotate until that obstacle is an human
                        print("Object from sonar is not a human")
                        if (abs(angle)<=10 and arduino.new_dist > 150): #the human is in front of the robot and eventually right/left osbstacle are far
                            print("Human in front of me - Approaching ")
                            functions_main.send_uno_lights(arduino.ser1, "move")
                            functions_main.send_initial_action_arduino("move", arduino.ser, "move")
                        elif angle >=10:
                                print("Human detected in right position...")
                                functions_main.send_uno_lights(arduino.ser1, "rotateRight")
                                functions_main.send_initial_action_arduino("rotateRight", arduino.ser, "none")
                        elif angle <= -10:
                            print("Human detected in left position...")
                            functions_main.send_uno_lights(arduino.ser1, "rotateLeft")
                            functions_main.send_initial_action_arduino("rotateLeft", arduino.ser, "none")
                else : #if there is no human
                    print("Searching ...")
                    functions_main.send_uno_lights(arduino.ser1, "rotateRight")
                    functions_main.send_initial_action_arduino("rotateRight", arduino.ser, "none")
                print("No object close")
                
            else:    
                if angle != 0: #there is a human in the FOV of robot
                    print("No object, but human in the FOV")
                    interaction = 1
                    if (abs(angle)<=10): #the human is in front of the robot
                        print("Approaching the human")
                        functions_main.send_uno_lights(arduino.ser1, "move")
                        functions_main.send_initial_action_arduino("move", arduino.ser, "move")
                    elif angle >=10:
                            print("Searching right...")
                            functions_main.send_uno_lights(arduino.ser1, "rotateRight")
                            functions_main.send_initial_action_arduino("rotateRight", arduino.ser, "none")
                    elif angle <= -10:
                        print("Searching left...")
                        functions_main.send_uno_lights(arduino.ser1, "rotateLeft")
                        functions_main.send_initial_action_arduino("rotateLeft", arduino.ser, "none")
                else: #if there is no human
                    print("Searching")
                    functions_main.send_uno_lights(arduino.ser1, "rotateRight")
                    functions_main.send_initial_action_arduino("rotateRight", arduino.ser, "none")
                    
        elif interaction == 2 and not JointAttention: # interaction = 2, so i'm starting the interaction loop
            print("INTERACTION LOOP")
            if time_out_system_hum > TIME_OUT_HUM: #If there is no human for too long
                print("INTERACTION LOOP - I've lost contact with the human")
                Finding_human = True # Am i looking for a human?
                time_out_system = 0 
                start_time_out_system = time.time()
                time_out_system_hum = 0
            elif time_out_system_hum <= TIME_OUT_HUM and time_out_system<TIME_OUT and Finding_human == False: 
                print("INTERACTION LOOP - Preparing the interaction")
                #If there is a human interacting and i'm inside the timeout
                time_out_system = 0
                print("Distance from sonar = {:.1f}".format(arduino.new_dist))
                if arduino.new_dist > 120.0: #if the distance to the chld is bigger than , get closer
                    tooCloseCount=0
                    tooFarCount += 1
                    if tooFarCount > 10:
                        tooFarCount = 0
                        if abs(angle)<=10 and angle !=0 :
                            print("INTERACTION LOOP - Child is far and in front ")
                            functions_main.send_uno_lights(arduino.ser1, "move")
                            functions_main.send_initial_action_arduino("move", arduino.ser, "move_find")
                        elif angle > 10:
                            print("INTERACTION LOOP - Child is on the right ")
                            functions_main.send_uno_lights(arduino.ser1, "rotateRight")
                            functions_main.send_initial_action_arduino("rotateRight", arduino.ser, "none")
                        elif angle < -10:
                            print("INTERACTION LOOP - Child is on the left ")
                            functions_main.send_uno_lights(arduino.ser1, "rotateLeft")
                            functions_main.send_initial_action_arduino("rotateLeft", arduino.ser, "none")
                elif arduino.new_dist < 40.0:
                    tooCloseCount += 1
                    tooFarCount = 0
                    if tooCloseCount > 10: #If I'm too cloose for too much time ( I need this time window in order to let the children interact with the robot)
                        print("INTERACTION LOOP - Too close")
                        functions_main.send_uno_lights(arduino.ser1, "move")
                        functions_main.send_initial_action_arduino("scared", arduino.ser, "move_find")
                        tooCloseCount = 0
                else: #if it's closer than 1.5m perform the interaction loop normally and select action of the child (child_action)
                    # Run Object Detection. I start now the timer for human time out because else comprehend 20 < dist 130 AND dist = Max dist, so no object sensed by the sonar
                    tooCloseCount = 0
                    tooFarCount = 0
                    current_time_out_system_hum = time.time()
                    time_out_system_hum = time_out_system_hum+(current_time_out_system_hum-start_time_out_system_hum)
                    start_time_out_system_hum = current_time_out_system_hum
                    
                    if angle != 0:
                        print("INTERACTION LOOP - Correctly interacting, waiting to receive an action")
                        time_out_system_hum = 0
                        if firstTime:
                                functions_main.send_uno_lights(arduino.ser1,"excited_attract")
                                functions_main.send_initial_action_arduino("excited_attract", arduino.ser, "excited_attract")
                                firstTime = False
                        if receiveAction:                                        
                            if child_action != "joint":
                                functions_main.decide_action(child_action) #decide robot behaviour based on action of the child and movement of the robot
                                functions_main.send_uno_lights(arduino.ser1, functions_main.current_action)
                                functions_main.send_initial_action_arduino( functions_main.current_action, arduino.ser, functions_main.current_action)
                                receiveAction = False
                            else:
                                JointAttention = True
                                start_time_JA = time.time()
                                duration_JA = 0
                    else: #if no human
                        if previousAngle > 0 and angle == 0: lookTo = "rotateRight"
                        elif previousAngle < 0 and angle == 0: lookTo = "rotateLeft"
                    print("Child Action: " + child_action + " | " + "Robot Action: " + functions_main.current_action)
                print("Time Out Human {:.1f} / 10 Sec".format(time_out_system_hum) )
            elif Finding_human == True and time_out_system < TIME_OUT : #if i need to find the child and i'm inside the timout
                print("INTERACTION LOOP - Looking for a human")
                #I need to find the human. I start counting for the general TIME_OUT
                # Run Object Detection
                current_time_out_system = time.time()
                time_out_system = time_out_system+(current_time_out_system-start_time_out_system)
                start_time_out_system = current_time_out_system
                print("Time out: {:.1f} / 40 ".format(time_out_system))
                
                if angle != 0:  # this can be replaced with a check if it is human, so this translate to if there is a human
                    print("INTERACTION LOOP - Human detecte in the FOV")        
                    Finding_human = False
                    functions_main.send_uno_lights(arduino.ser1, "excited_attract")
                    functions_main.send_initial_action_arduino("excited_attract", arduino.ser, "excited_attract")
                    time_out_system_hum = 0   
                    time_out_system = 0                     
                else:
                    print("INTERACTION LOOP - Searching")
                    functions_main.send_uno_lights(arduino.ser1, "none")
                    functions_main.send_initial_action_arduino(lookTo, arduino.ser, "none")
            elif Finding_human == True and time_out_system>TIME_OUT: #If 'm looking for the children and i run out of time
                print("Terminating the program")
                child_action = "QUIT"
                
        ####-----END HUMAN INTERACTION----####
        
        oldTargetBox = targetBox.copy()
        
        t2 = time.perf_counter()  
        elapsedTime = t2-t1      
        fps = 1/elapsedTime      
        i=0
       
        if args.no_show:
            print("Cicli al secondo: {:.1f} ".format(float(fps)))
            continue
                
        if child_action == "QUIT":
            if duration_JA != 0 and duration_LOOKING != 0:
                now = datetime.now()
                dt_string = now.strftime("%d/%m/%y %H:%M:%S")
                data = dt_string + ',' + str(duration_JA) + ',' + str(duration_LOOKING) + '\n'
                with open('experiment_records.csv','a') as fp:
                    print("Record stored successfully")
                    fp.write(data)  
            break
        
        #visualization
        for bbox in bboxes:
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[0] + bbox[2], bbox[1] + bbox[3]), color, 1)
            if len(labels_detected)>0:
                cv2.putText(frame, labels[labels_detected[i].astype(int)], (bbox[0]+3,bbox[1]-7), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255))                
                cv2.putText(frame, "{:.3f}".format(score_detected[i]), (bbox[0]+3,bbox[1]+7), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255))
                i+=1 #indent at the same level of putText
        
        cv2.putText(frame, "FPS: {:.1f}".format(float(fps)), (5,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    
        cv2.imshow('Demo', frame)
        
        key = cv2.waitKey(1)            
        if key == 27:
            if duration_JA != 0 and duration_LOOKING != 0:
                now = datetime.now()
                dt_string = now.strftime("%d/%m/%y %H:%M:%S")
                data = dt_string + ',' + str(duration_JA) + ',' + str(duration_LOOKING) + '\n'
                with open('experiment_records.csv','a') as fp:
                    print("Record stored successfully")
                    fp.write(data)  
            break
      
        
        
    vs.stop()
    #datas.stop()

if __name__ == "__main__":
    args = build_argparser().parse_args()
    run_demo(args)
