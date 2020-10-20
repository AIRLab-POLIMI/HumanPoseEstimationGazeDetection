
Be sure to install correctly OpenVino environment and EdgeTPU. Follow the documentation provided by Intel and Google.

In order to run the Gaze Estimation model it is needed at least Tensorflow 2.2. Be sure to check http://github.com/PINTO0309/Tensorflow-bin

In order to be sure that everything works fine, connect the USB HUB with NCS2 and Google Coral only after the boot of the Raspberry.

Running the applications with the `-h` option yields the following usage messages:

# detection_estimation_myriad_coral.py

This is the debug script for visualization and extraction of information from all the models that are running simultaneously.
It always make available the theorical istantaneous FPS for Object Detection and for Pose Estimation + Gaze Estimation

```
usage: detect_estimation_myriad_coral.py [-h] [-m_od MODEL_OD]
                                            [-m_hpe MODEL_HPE]
                                            [-i INPUT]
                                            [-d DEVICE]
                                            [--person_label PERSON_LABEL]
                                            [--modality]
                                            [--no_show]

optional arguments:

  -h, --help            show this help message and exit
  -m_od MODEL_OD, --model_od MODEL_OD
                        path to model of object detector in xml format, inferenced by NCS2. Default is ssdlite_mobilenet_v2
                        trained on COCO, precision FP16
  -m_hpe MODEL_HPE, --model_hpe MODEL_HPE
                        path to model of human pose estimator in tflite format, inferenced by Coral TPU.                      
  -i INPUT [INPUT ...], --input INPUT [INPUT ...] 
                        Set by default to 0, meaning PiCamera. Other videos are not compatible due to ffmpeg incompatibilities
  -d DEVICE, --device DEVICE
                        Set by default to MYRIAD. It specify where to perform inference for Object detection model
  --person_label PERSON_LABEL
                        Label of class person for detector and depends by the net. For mobilenet-ssd.xml and ssdlite_mobilenet_v2 is 1.
                        It is needed to extract only the boundaries of "person" entity
  --modality            Set by default to "Multi". It specify if I only want to visualize all the estimations from the main person (the biggest box
                        among the person's boxes) or if I want to visualize every skeleton+gaze available in the FOV.
  --no_show             Optional. Do not display visual output, only text with fps performance.
  
```
# main_program.py

This is the script that actually implement all the control loop designed for the behavior of the robot. See the informations above about arguments.
The only change is that the --no_show argument needs to be passed: visualization of the camera is not yet implemented.

During the script, if you are in the INTERACTION LOOP, you can simulate the actions that the child could do with the robot.

Press: 'a' -> Touch, 's' -> push, 'd' -> hit, 'f' -> hug, 'g' -> strongHug, 'h' -> none, 'j' -> JointAttention.

The first action that is sent is a bit buggy, so you should press a couple of times. JointAttention state is under development, but it correctly
extract the data from the gaze estimation models.

## detector.py

All the credits to INTEL OpenVINO Toolkit resources.
It manages the inference of mobilenet-ssd.xml and ssdlite_mobilenet_v2 on MYRIAD device.

## pose_engine.py

Al the credits to Tensorflow - Google Coral resources.
It manages the inference of posenet on Google Coral TPU.

## finalmodel.py

All the credits to Philipe Dias, Paris Her. Check them @ https://bitbucket.org/phil-dias/gaze-estimation/src/master
Based on their [paper](https://openacces.thecvf.com/content_WACV_2020/papers/Dias_Gaze_Estimation_for_Assisted_Living_Environments_WACV_2020_paper.pdf)
It creates model and manages the weights of the net that is run locally on Raspberry.

## functions_main.py

Credits to Ane San Martin.
It manages all the functions that involve the communication with Arduino (outward) and the functions that manages the
correct response based on the previous action of the child. Also sounds are managed here.

## connections_arduinos.py

Credits to Ane San Martin.
It manages the initialization of the communication between both Arduino Uno/Mega and Raspberry. Also manages all the
inward communication from sonars of the Mega and its status: position of the obstacle (left, front, right), distance, movement 
of the robot (forward, backward, stopped)
