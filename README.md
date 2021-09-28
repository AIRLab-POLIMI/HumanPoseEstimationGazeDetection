
This is the work done by Dario Sortino, advised by prof.Andrea Bonarini to detect pose of a kid and gaze direction towards a recognized object to implement shared attention mechanisms with children with ASD.

Be sure to install correctly OpenVino environment and EdgeTPU. Follow the documentation provided by Intel and Google.

In order to run the Gaze Estimation model it is needed at least Tensorflow 2.2. Be sure to check http://github.com/PINTO0309/Tensorflow-bin

In order to be sure that everything works fine, connect the USB HUB with NCS2 and Google Coral only after the boot of the Raspberry.

# detection_estimation_myriad_coral.py
Run the script using python3

This is the debug script for visualization and extraction of information from all the models that are running simultaneously.
It always makes available the theorical istantaneous FPS for Object Detection and for Pose Estimation + Gaze Estimation. Arguments can be ignored.

# main_program.py
Run the script using python3

This is the script that actually implement all the control loop designed for the behavior of the robot. Arguments can be ignored.

During the script, if you are in the INTERACTION LOOP, you can simulate the actions that the child could do with the robot.

Press: 'a' -> Touch, 's' -> push, 'd' -> hit, 'f' -> hug, 'g' -> strongHug, 'h' -> none, 'j' -> JointAttention ('k' -> end JA program)

The first action that is sent won't be recognized, so press twice.

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

It manages all the functions that involve the communication with Arduino (outward) and the functions that manages the
correct response based on the previous action of the child. Also sounds are managed here.

## connections_arduinos.py

It manages the initialization of the communication between both Arduino Uno/Mega and Raspberry. Also manages all the
inward communication from sonars of the Mega and its status: position of the obstacle (left, front, right), distance, movement 
of the robot (forward, backward, stopped)
