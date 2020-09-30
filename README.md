Be sure to install correctly OpenVino and EdgeTPU. Follow the documentation provided by Intel and Google.

Running the application with the `-h` option yields the following usage message:
```
usage: single_human_pose_estimation_demo.py [-h] -m_od MODEL_OD -m_hpe
                                            MODEL_HPE [-i INPUT [INPUT ...]]
                                            [-d DEVICE]
                                            [--person_label PERSON_LABEL]
                                            [--no_show]
                                            [-u UTILIZATION_MONITORS]

optional arguments:
  -h, --help            show this help message and exit
  -m_od MODEL_OD, --model_od MODEL_OD
                        path to model of object detector in xml format, inferenced by NCS2
  -m_hpe MODEL_HPE, --model_hpe MODEL_HPE
                        path to model of human pose estimator in tflite format, inferenced by Coral TPU
  -i INPUT [INPUT ...], --input INPUT [INPUT ...]
                        path to video or image/images, 0 is for picamera
  -d DEVICE, --device DEVICE
                        Specify the target to infer on CPU, GPU, or MYRIAD. type MYRIAD when working with NCS2
  --person_label PERSON_LABEL
                        Label of class person for detector. Depends by the net. For mobilenet-ssd.xml is 1.
  --no_show             Optional. Do not display visual output, only text with fps performance.
```
