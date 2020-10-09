#!/usr/bin/env python3

import argparse
import sys
import cv2
import time
import numpy as np
import math

from edgetpu.basic import edgetpu_utils
from pose_engine import PoseEngine
from finalmodel import prepare_modelSingle

from openvino.inference_engine import IECore

from detector import Detector

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
    parser.add_argument("-m_od", "--model_od", type=str, default= "ssdlite_mobilenet_v2/FP16/ssdlite_mobilenet_v2.xml",
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

class ImageReader(object):
    def __init__(self, file_names):
        self.file_names = file_names
        self.max_idx = len(file_names)

    def __iter__(self):
        self.idx = 0
        return self

    def __next__(self):
        if self.idx == self.max_idx:
            raise StopIteration
        img = cv2.imread(self.file_names[self.idx], cv2.IMREAD_COLOR)
        if img.size == 0:
            raise IOError('Image {} cannot be read'.format(self.file_names[self.idx]))
        self.idx += 1
        return img


class VideoReader(object):
    def __init__(self, file_name, width, height):
        try:
            self.file_name = int(file_name[0])
        except:
            self.file_name = file_name[0]
        self.width = width
        self.height = height


    def __iter__(self):
        self.cap = cv2.VideoCapture(self.file_name)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.height)
        if not self.cap.isOpened():
            raise IOError('Video {} cannot be opened'.format(self.file_name))
        return self

    def __next__(self):
        was_read, img = self.cap.read()
        if not was_read:
            raise StopIteration
        return img

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
        return result
    
      
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
    
    if args.model_od == "mobilenet-ssd.xml":
        labels = ['Background','Person','Car', 'Bus', 'Bicycle','Motorcycle'] #???
    elif args.model_od == "ssdlite_mobilenet_v2/FP16/ssdlite_mobilenet_v2.xml" or "ssdlite_mobilenet_v2/FP32/ssdlite_mobilenet_v2.xml" :
        labels = [" ",]
        file1 = open("labels.txt", 'r')
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
    model_gaze.load_weights('/home/pi/Detection-and-Human-Pose-Estimation---RASPBERRY/trainedOnGazeFollow_weights.h5')    
    print("#-----Done-----#")
        
    if args.input != '':
        img = cv2.imread(args.input[0], cv2.IMREAD_COLOR)
        frames_reader, delay = (VideoReader(args.input, camera_width, camera_height), 1) if img is None else (ImageReader(args.input), 0)
    else:
        raise ValueError('--input has to be set')
        
    framecount = 0
    detectframecount = 0
    estimation_fps = ""
    detection_fps = ""
    fps = ""
    time1 = 0
    time2 = 0
    imdraw = []
    
    for frame in frames_reader:
        
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
                imdraw = elaborate_gaze(imdraw, head, scores_head, model_gaze)
                
        else:
            imdraw = color_image
        
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
        cv2.imshow('Demo', imdraw)
        key = cv2.waitKey(delay)
        if key == 27:
            break

if __name__ == "__main__":
    args = build_argparser().parse_args()
    run_demo(args)
