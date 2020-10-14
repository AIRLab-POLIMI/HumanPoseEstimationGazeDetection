#!/usr/bin/env python3

import argparse
import sys
import cv2
import time
import numpy as np
import math
from threading import Thread

from edgetpu.basic import edgetpu_utils
from pose_engine import PoseEngine
from finalmodel import prepare_modelSingle
from openvino.inference_engine import IECore
from detector import Detector

import functions_main
import connections_arduinos as arduino #new_user_fun

import socket

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



def draw_on_img(img, centr, res, uncertainty): #gaze drawing
    res[0] *= img.shape[0]
    res[1] *= img.shape[1]
    angle = -math.degrees(math.atan2(res[1],res[0]))

    norm1 = res / np.linalg.norm(res)
    norm1[0] *= img.shape[0]*0.10
    norm1[1] *= img.shape[0]*0.10
    
    point = centr + norm1 

    result = cv2.arrowedLine(img, (int(centr[0]),int(centr[1])), (int(point[0]),int(point[1])), (0,0,0), thickness=3, tipLength=0.2)
    result = cv2.arrowedLine(result, (int(centr[0]),int(centr[1])), (int(point[0]),int(point[1])), (0,0,255), thickness=2, tipLength=0.2)
    result = cv2.putText(result, " Gaze Uncertainty {:.3f}".format(uncertainty), (10,450), cv2.FONT_HERSHEY_SIMPLEX ,0.5, (0,255,0),1)    
    result = cv2.putText(result, " Gaze Angle {:.3f}".format(angle), (10,470), cv2.FONT_HERSHEY_SIMPLEX ,0.5, (0,255,0),1)
    return result

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
        result = draw_on_img(img, Centroids, gazeDirections, Uncertainties)
        return result, pred_
    
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
    
def human_camera_angle(person, im_width):
    xmin = person[0]
    xmax = person[0] + person[2]
    left = xmin*im_width
    right = xmax*im_width
    centerx = left + ((right-left)/2)
    angle = (0.09375*centerx)-22.5
    return angle


def run_demo(args):
    

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
    
    #Loading labels based on the Object Detector model
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
    print("#-----Done-----#")
    
    #Mobilenet - NCS2 Inference
    print("#-----Loading Mobilenet - NCS2 Inference -----#")
    ie = IECore()
    detector_person = Detector(ie, path_to_model_xml=args.model_od, device=args.device, label_class=args.person_label)
    print("#-----Done-----#")
    

    #Custom Gaze Estimator predictor
    print("#-----Loading Gaze Estimator Net -----#")    
    model_gaze = prepare_modelSingle('relu')
    model_gaze.load_weights('/home/pi/Detection-and-Human-Pose-Estimation---RASPBERRY/models/trainedOnGazeFollow_weights.h5')    
    print("#-----Done-----#")
    
    #Framerate variables
    framecount = 0
    detectframecount = 0
    estimation_fps = ""
    detection_fps = ""
    fps = ""
    time1 = 0
    time2 = 0
    imdraw = []
    
    #Human Interaction variables
    TIME_OUT = 20 #The robot have 20 sec to find the user if he/she is gone during the interaction
    TIME_OUT_HUM = 120 #every 120 sec i need to check if i'm dealing with a human
    
    tracking_a_user = False #is the obstacle i read from sonar an human?
    Finding_human = False #am i looking for a human?
    actual_action = " "
    action_sound = "none"
    child_action = "none"
    
    interaction = 0
    cont_int = 0
    action_count = 1
    
    #--> Counting the time that manages the reseach of a human
    time_out_system = 0
    start_time_out_system = 0
    current_time_out_system = 0
    
    #--> Counting the time of interaction
    time_out_system_hum = 0
    start_time_out_system_hum = 0
    current_time_out_system_hum = 0
    
    count_find = 0
    
    #Camera Thread
    vs = WebcamVideoStream(camera_width, camera_height, src=0).start()
    
    functions_main.send_uno_lights(arduino.ser1,"none")
    
    while True:
        
        key = cv2.waitKey(1)            
        if key == 27:
            break
        
        if key == ord('1'):
            child_action = "touch"
            print("action = "+child_action)
        elif key == ord('2'):
            child_action = "push"
            print("action = "+child_action)
        elif key == ord('3'):
            child_action = "hit"
            print("action = "+child_action)
        elif key == ord('4'):
            child_action = "hug"
            print("action = "+child_action)
        elif key == ord('5'):
            child_action = "strongHug"
            print("action = "+child_action)
        elif key == ord('6'):
            child_action = "none"
            print("action = "+child_action)
            
        
        
        frame = vs.read()
        t1 = time.perf_counter()

        # Run Object Detection
        bboxes, labels_detected, score_detected, bboxes_person = detector_person.detect(frame)
        
        main_person = [0,0,0,0]
        areas=[]
        for bbox in bboxes_person:
            area = bbox[2]*bbox[3]
            areas.append(area)
        if areas:
            box_person_num = areas.index(max(areas))
            main_person = [bboxes_person[box_person_num][0],bboxes_person[box_person_num][1],bboxes_person[box_person_num][2],bboxes_person[box_person_num][3]]
            angle = human_camera_angle(main_person, camera_width)
        else:
            angle = 0
            
        # Run Pose + Gaze Estimation
        color_image = frame
        color_image = cv2.resize(color_image, (model_width, model_height))
        prepimg = color_image[:, :, ::-1].copy()         
        res, inference_time = engine.DetectPosesInImage(prepimg)
        
        if res:
            detectframecount += 1            
            head, scores_head = elaborate_pose(res)
            
            if args.no_show:
                prediction = elaborate_gaze(imdraw, head, scores_head, model_gaze)
            else:
                imdraw = overlay_on_image(color_image, res, model_width, model_height, main_person, args.modality)
                imdraw, prediction = elaborate_gaze(imdraw, head, scores_head, model_gaze)
                
        else:
            imdraw = color_image
            
        ####-----START HUMAN INTERACTION-----####
        
        angle_tot = 0
        angle_mean = 0
        count = 0
        
        arduino.new_user_function() #Connect with the Mega and obtain data from sensors
        if arduino.state_system == "busy":
            print("System is busy")
        else:
            #interaction = 0 or interaction=1 is when the system is trying to estabilish an interaction with the child
            #interaction = 2 is when the robot is already interacting with the human
                        
            if interaction != 2 : #If I'm not interacting with the human
                print("Interaction != 2, I'm not interacting with the human")
                arduino.previous_action = "none"
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
                            action_sound = "excited"
                            Finding_human = False
                        else: #if it find an object that is not a human, must rotate until that obstacle is an human
                            print("Object from sonar is not a human")
                            cont_int = 0
                            if angle >=0:
                                print("Searching right...")
                                functions_main.send_uno_lights(arduino.ser1, "none")
                                functions_main.send_initial_action_arduino("rotateRight", arduino.ser, "none")
                            else:
                                print("Searching left...")
                                functions_main.send_uno_lights(arduino.ser1, "none")
                                functions_main.send_initial_action_arduino("rotateLeft", arduino.ser, "none")
                    else : #if there is no human
                        print("No human in the FOV, searching")
                        functions_main.send_uno_lights(arduino.ser1, "rotate")
                        functions_main.send_initial_action_arduino("rotate", arduino.ser, "found")
                        action_sound = "found"
                        #should reproduce sound?
                else : #if there is no object detected by the sonar
                    print("No object close")
                    cont_int = 0
                    if angle != 0: #there is a human in the FOV of robot
                        print("No object, but human in the FOV")
                        interaction = 1
                        if (abs(angle)<7.5): #the human is in front of the robot
                            print("Approaching the human")
                            functions_main.send_uno_lights(arduino.ser1, "move")
                            functions_main.send_initial_action_arduino("move", arduino.ser, "move")
                            action_sound = "move"
                            #reproduce sound??
                        else:
                            if angle >=0:
                                print("Searching right...")
                                functions_main.send_uno_lights(arduino.ser1, "none")
                                functions_main.send_initial_action_arduino("rotateRight", arduino.ser, "none")
                            else:
                                print("Searching left...")
                                functions_main.send_uno_lights(arduino.ser1, "none")
                                functions_main.send_initial_action_arduino("rotateLeft", arduino.ser, "none")
                    else: #if there is no human
                        functions_main.send_uno_lights(arduino.ser1, "rotate")
                        functions_main.send_initial_action_arduino("rotate", arduino.ser, "found")
                        action_sound = "found"
                        print("No human in the FOV, searching")
                        #should reproduce sound?
            else: # interaction = 2, so i'm starting the interaction loop
                print("INTERACTION LOOP")
                if arduino.old_user == "none" and arduino.previous_old_user=="none": #I've lost the interaction with the human
                    print("INTERACTION LOOP - I've lost contact with the human")
                    time_out_system = 0
                    Finding_human = True # Am i looking for a human?
                    start_time_out_system = time.time()
                    time_out_system_hum = 0
                    count_find = 0
                elif arduino.old_user != "none" and time_out_system<TIME_OUT:
                    print("INTERACTION LOOP - Preparing the interaction")
                    #if i'm still interacting with the child and inside the timeout time
                    if Finding_human == False: #if i was not looking for a human
                        current_time_out_system_hum = time.time()
                        time_out_system_hum = time_out_system_hum+(current_time_out_system_hum-start_time_out_system_hum)
                        start_time_out_system_hum = current_time_out_system_hum
                        if time_out_system_hum>TIME_OUT_HUM : #if i'm over the timeouthuman, check if it's a human
                            time_out_system_hum = 0
                            Finding_human = True
                        else:
                            time_out_system = 0
                            if arduino.new_dist > 150.0: #if the distance to the chld is bigger than , get closer
                                print("INTERACTION LOOP - Approaching the child")
                                if arduino.old_user != "front" :
                                    if arduino.old_user == "right":
                                        functions_main.send_uno_lights(arduino.ser1, "none")
                                        functions_main.send_initial_action_arduino("rotateRight", arduino.ser, "none")
                                    else:
                                        functions_main.send_uno_lights(arduino.ser1, "none")
                                        functions_main.send_initial_action_arduino("rotateLeft", arduino.ser, "none")
                                else: #if it's aligned, move forward
                                    functions_main.send_uno_lights(arduino.ser1, "move")
                                    functions_main.send_initial_action_arduino("move", arduino.ser, "move_find")
                            else: #if it's closer than 1.5m perform the interaction loop normally and select action of the child (child_action)
                                print("INTERACTION LOOP - Correctly interacting")
                                functions_main.decide_action(child_action, arduino.user_movement) #decide robot behaviour based on action of the child and movement of the robot
                                functions_main.send_uno_lights(arduino.ser1, functions_main.current_action)
                                functions_main.send_initial_action_arduino( functions_main.current_action, arduino.ser, functions_main.current_action)
                                print(child_action + functions_main.current_action)
                    else: #i need to find the human
                        print("INTERACTION LOOP - Looking for a human")
                        if angle != 0:  # this can be replaced with a check if it is human, so this translate to if there is a human 
                            print("INTERACTION LOOP - Human detecte in the FOV")
                            count = 4
                            tracking_a_user = functions_main.human_verification(angle, arduino.old_user, count) #it check if obstacle from sonar is a human
                            if tracking_a_user == True:
                                print("INTERACTION LOOP - Object from sonar is a human")
                                interaction = 1
                                ### --- I Should check if the path between the robot and the child is clear
                                ##if the human is free
                                ##if the human is free for few instant
                                interaction = 2
                                start_time_out_system_hum = time.time()
                                action_sound = "excited"
                                Finding_human = False
                            else: #if it find an object that is not a human, must rotate until that obstacle is an human
                                print("INTERACTION LOOP - Object from sonar is not a human")
                                cont_int = 0
                                if angle >=0:
                                    print("INTERACTION LOOP - Searching right...")
                                    functions_main.send_uno_lights(arduino.ser1, "none")
                                    functions_main.send_initial_action_arduino("rotateRight", arduino.ser, "none")
                                else:
                                    print("INTERACTION LOOP - Searching left...")
                                    functions_main.send_uno_lights(arduino.ser1, "none")
                                    functions_main.send_initial_action_arduino("rotateLeft", arduino.ser, "none")
                                start_time_out_system = time.time()
                        else: #if there is no human
                            print("INTERACTION LOOP - No human in the FOV, searching")
                            current_time_out_system = time.time()
                            time_out_system = time_out_system + (current_time_out_system - start_time_out_system)
                            start_time_out_system = current_time_out_system
                            functions_main.send_uno_lights(arduino.ser1, "rotate")
                            functions_main.send_initial_action_arduino("rotate", arduino.ser, "found")
                            action_sound = "found"
                            #should reproduce sound?
                elif Finding_human == True and time_out_system < TIME_OUT : #if i need to find the child and i'm inside the timout
                    print("INTERACTION LOOP - Looking for a human")
                    #I need to find the human
                    if angle != 0:  # this can be replaced with a check if it is human, so this translate to if there is a human
                        print("INTERACTION LOOP - Human detecte in the FOV")
                        count = 4
                        tracking_a_user = functions_main.human_verification(angle, arduino.old_user, count) #it check if obstacle from sonar is a human
                        if tracking_a_user == True:
                            print("INTERACTION LOOP - Object from sonar is a human")
                            interaction = 1
                            ### --- I Should check if the path between the robot and the child is clear
                            ##if the human is free
                            ##if the human is free for few instant
                            interaction = 2
                            start_time_out_system_hum = time.time()
                            action_sound = "excited"
                            Finding_human = False
                        else: #if it find an object that is not a human, must rotate until that obstacle is an human
                            print("INTERACTION LOOP - Object from sonar is not a human")
                            cont_int = 0
                            if angle >=0:
                                print("INTERACTION LOOP - Searching right...")
                                functions_main.send_uno_lights(arduino.ser1, "none")
                                functions_main.send_initial_action_arduino("rotateRight", arduino.ser, "none")
                            else:
                                print("INTERACTION LOOP - Searching left...")
                                functions_main.send_uno_lights(arduino.ser1, "none")
                                functions_main.send_initial_action_arduino("rotateLeft", arduino.ser, "none")
                            start_time_out_system = time.time()
                    else: #if there is no human
                        print("INTERACTION LOOP - No human in the FOV, searching")
                        current_time_out_system = time.time()
                        time_out_system = time_out_system + (current_time_out_system - start_time_out_system)
                        start_time_out_system = current_time_out_system
                        functions_main.send_uno_lights(arduino.ser1, "rotate")
                        functions_main.send_initial_action_arduino("rotate", arduino.ser, "found")
                        action_sound = "found"
                        #should reproduce sound?
                elif Finding_human == True and time_out_system>TIME_OUT: #If 'm looking for the children and i run out of time
                    print("Going out of the interaction loop")
                    current_time_out_system = 0
                    time_out_system = 0
                    start_time_out_system = 0
                    Finding_human == False
                    interaction = 0
                    
        ####-----END HUMAN INTERACTION----####
                
        i=0
        # FPS Calculation
        framecount += 1
        if framecount >= 15:
            fps       = "Overall: {:.1f} FPS, ".format(float(time1/15))
            estimation_fps = "Pose + Gaze Est.: {:.1f} FPS, ".format(float(detectframecount/time2))
            framecount = 0
            detectframecount = 0
            time1 = 0
            time2 = 0
        t2 = time.perf_counter()
        elapsedTime = t2-t1
        time1 += 1/elapsedTime
        time2 += elapsedTime
        detection_fps = "Detection: {:.1f} FPS".format(float(1/detector_person.infer_time))
        display_fps = fps + estimation_fps + detection_fps

        if args.no_show:
            print(display_fps)
            continue 
        
        #Visualization
        for bbox in bboxes:
            cv2.rectangle(imdraw, (bbox[0], bbox[1]), (bbox[0] + bbox[2], bbox[1] + bbox[3]), (0, 0, 255), 1)
            if len(labels_detected)>0:
                cv2.putText(imdraw, labels[labels_detected[i].astype(int)], (bbox[0]+3,bbox[1]-7), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255))
        i+=1
        
        cv2.putText(imdraw, display_fps, (5,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        cv2.putText(imdraw, "Angle of person: {:.1f} ".format(angle), (5,40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        cv2.imshow('Demo', imdraw)
        
            
    vs.stop()

if __name__ == "__main__":
    args = build_argparser().parse_args()
    run_demo(args)
