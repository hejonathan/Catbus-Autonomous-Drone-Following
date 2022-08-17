# Cat(kin)bus

Link to slide presentation: https://docs.google.com/presentation/d/1UctnK75cxS3Pep-5C0489GU4yb0VlhBdIWLTOmArEKs/edit?usp=sharing

Charlie Benjamin, 
Jonathan He,
Yuta Roy Ioriya,
Gracie Sheng.

Beaver Works Summer Institute

UAV Course

Final Challenge Report

August 7, 2022


## 


## Acknowledgements

We would like to express our gratitude to the Beaver Works Summer Institute for providing us with equipment such as Tello EDU drones, UNO R3 Project kits, and custom backpacks as well as daily instruction in preparation for the final challenge. In particular, we would like to thank the instructors and teaching assistants of the UAV course: Nathaniel Hanson, Aryk Ledet, Matthew Boyd, Ifueko Igbinedion, Rumaisa Abdulhai, and Matthew Schofield.

## Introduction

Interest in unmanned aerial vehicles (UAVs) has skyrocketed in recent years with the advent of accessible and affordable commercial off-the-shelf drones. This appeal has led to the implementation of UAVs for a wide range of military, commercial, and hobbyist purposes.


## Brainstorming

We originally came up with 3 main ideas that we then individually ranked in terms of preference. The first idea was to have the drone navigate in front of a person’s face and read their emotions, from which point the drone would exhibit some sort of reaction. The second idea was to have the drone map and model a given trail. The third idea was to have the drone follow a person and be able to follow a person and record the exact path that they took. 

Finally, we settled on the drone following a person’s head and recording the path of the drone through exact xyz and coordinates and yaw.

![](https://lh3.googleusercontent.com/Iw9o5hTorVMSiuXhJ3XjoAkY1wFGRJx1cZKd2M5P7vLYow26SdG3iZ0-4tlt3aXnSIJEpF2qT2ipVbt5C59r1OTfhdmuMzSc-Ed-IwaVa5tWg5ncD1BIwS6KvLO19YkebOD2MwOpJMuPbhLJG0bUHfg)![](https://lh4.googleusercontent.com/GIIofhqJl_Dey_DqUPRHw-aaVEbf3lZ1SAqcMnIO_MwhpjfJLU8qddu5PYmYz_q6mtL-X6hG16y-236KmR8idl_PVZkrkehDlWnf53olIEHXfcSYBWYWCb4Ve-twvT5MN_A-WMIC-UKo2NO9tPVbNtw)![](https://lh4.googleusercontent.com/Jt3DhRN7MX4VApuZRDvMEZcXfqbJ3Aassi16PKywatUyv5NxkvRPvrHS58ufl8Fu-yyUnKtFzka412hYbLBj3m5NuUlduiAUNJyr8o9v1D7nL-YM3UxlScI8ehE3CjeMvaePd1KOm-NwUSTsdosc4gY)

## Purpose

The purpose of this project is to combine existing applications of drones in surveillance and tracking, together with knowledge of machine learning techniques, to develop a feature which would allow the user to explore an environment hands-free, capture video feed, and match the video to its precise location in the path traversed. This project investigates the autonomous capabilities of drones in unfamiliar environments, human-robot interaction, and artificial intelligence.


## Objectives

1. The drone shall identify a person by their head (or by a marker attached to their body) and follow them while mapping their trail. 
2. The drone should then be able to navigate the same trail in the opposite direction to lead the user back to the original location.


## Hardware and Software Tools


### Hardware

![](https://lh6.googleusercontent.com/ShK7EKGWd9dJj_eMyhmEoBA7-0BE5rl8bIJSyYPHqnX8bo7h4tdeH1lfGaveOm_R1wSlTb9WmadX6SLh7lvGR30bz7KUwHaz3zf38-AhuZQwoF2paJbJdfE4SJSZERGsAg_JXDgPSm6T0YlQHYl08Jk)

For the purposes of the project, the Ryze Tello EDU drone was used to achieve the engineering objectives. The Tello EDU is a miniature quadrotor that may be programmed in Python, Swift, or Scratch. Python was used as the primary programming language in this project. Below is a list of aircraft specifications for the Tello EDU: 

Weight: 87g (Propellers and Battery Included)

Dimensions: 98×92.5×41 mm

Propeller: 3 inches

Built-in Functions: Range Finder, Barometer, LED, Vision System, Wi-Fi, 720p Live View

Source: https&#x3A;//www.ryzerobotics.com/tello-edu/specs

Furthermore, the forward camera of the Tello was used to collect video footage, ArUco markers with dimensions 5in x 5in were used for preliminary localization of the drone within the environment, and personal computers and laptops served the purpose of ground control stations during testing.




### Software

Djitellopy: a python interface for the Tello drone

Retrieved from <https://github.com/damiafuentes/DJITelloPy>

ROS: a robotics middleware suite that contains software libraries and tools used to construct robot applications

Retrieved from [https://www.ros.org](https://www.ros.org/)

RViz: a 2D/3D visualization tool for ROS applications

Retrieved from <https://github.com/ros-visualization/rviz>

Numpy: a Python package that facilitates matrix operations and linear algebra

Retrieved from [https://numpy.org](https://numpy.org/)

PyTorch: a Python package that uses tensor computation and GPU acceleration; implements machine learning based on the Torch library

Retrieved from [https://pytorch.org](https://pytorch.org/)

Yolov5 Pre-trained Head and Person Detection Model

Retrieved from <https://github.com/deepakcrk/yolov5-crowdhuman>

OpenCV: a powerful computer vision library; contains detection methods for ArUco markers

Retrieved from [https://opencv.org](https://opencv.org/)


### ROS Architecture

The ROS Architecture of our project consists of 5 nodes. Below are the nodes with descriptions and the topics which they published and subscribed to:

![](https://lh6.googleusercontent.com/EGaRML7jTUXCMljuO_nkawJLXAdm8VlQe3buLZ9kHAOK8S2TKRFSEbdWXr-VN_LzfSMr2gm4lU1TTMZY7VuvY3UCH3dVLy67As0J81Wh79l4Mgn5RTvA4ZjrWr1e5sip5g48SzklvA4WdJUTPkV7FA)

**Driver**:

- Subscribes to velocities from motion control node and sends appropriate commands to the drone
- Publishes video feed to screen
- Allows keyboard teleoperation for takeoff and landing

**Localization**:

- Precondition: Have multiple ArUco markers and know the displacements between them
- Subscribes to video feed from driver node
- Calculates Eurler angles and rotation matrices
- Publishes drone pose relative to markers (FOV of any one marker) to motion control node

**YOLO**:

- Precondition: a robust head detection model that outputs the coordinates of the head in pixels
- Subscribes to video feed from diver node
- Calculates the displacement of the head relative to the drone’s camera
- Publishes the displacement (FOV of the drone) to motion control node

**Motion Control + RViz**:

- Subscribes to pose messages from localization node and head displacement from YOLO node
- Calculates the location of the head in aruco tag’s FOV
- Adds the head’s location to list of points in desired trajectory
- Appends list of points to path for visualization in RViz subnode
- Implements PID control to determine appropriate trajectories
- Publishes velocities to driver node


## Day-by-Day Plan

**Monday:** Finish plan, start building driver and localization nodes, set up catkin

**Tuesday:** Finish building rough drafts of every node

**Wednesday:** Finalize nodes and start testing 

**Thursday:** Full day of testing and revisions, work on documentation

**Friday: **Same as Thursday and adjust project as needed

**Saturday:** Prepare and practice presentation 

**Sunday:** Present our project and wow the crowd


## Procedure

**Mapping:** 

- Given camera feed, determine the 3D trail of the user’s head 
- For every 5 frames, identify and locate the person using Yolo (in terms of pixels in the frame)
- Calculate the person’s location in respect to the current location of the drone (in terms of distance irl)
- Calculate the person’s location in respect to the starting position of the drone
- Add the result to the path array
- Keep adding to the list until keyboard interruption

**Following: **

- As the drone moves around, keep track of its displacement from the starting position
- Given a list of points, find a way to follow these points in a natural manner (using either way-point of xyz speed)
- Keep following until the end of the list

**Backtracking: **

- Turn 180 degrees in yaw
- Reverse the list of points
- Use the same algorithm in step 2 to traverse the reversed list of trajectory points


## Testing

Driver Node: No issues. The Driver node ran perfectly the first time we launched which was wonderful and made the testing process much easier for the other nodes.

Motion Control Node: The initial solution of using Cubic Spline for waypoint navigation presented difficult challenges for short term implementation, so we fell back to using the PID solution. There was an issue with the motions being too aggressive at first, but it was ultimately resolved after tuning the PID constants. 

Localization Node: No issues since we had experience during the practicals. However, we were facing inconsistent detections which we eventually resolved by using our back-up option: dead-reckoning.

RViz Node: No issues with the actual code and publishing to the topic. However, we did run into some trouble figuring out how to display the model. This was soon cleared up once we realized that some of the online tutorials were using Indigo and not Noetic.

 

Yolo Node: We faced issues running YOLOv5 in the ROS framework, which was resolved after adding a setup file in the package root. We then had problems with OpenBLAS unable to use multi-processing since the VM did not have OpenMP natively and thus the detection runs very slow at 1/11 FPS. This was resolved by disabling multi-processing which brought the frame rate up to ½ FPS. However, this was still not ideal, so we conducted inference on a resized smaller video stream. This process drastically improved the runtime to 3 FPS while having minimal impact on accuracy.

Miscellaneous: We were having trouble translating everything from the drone’s FOV to the ArUco tag’s stationary point of view. This was resolved after looking up information online. Additionally, the ArUco tag detection was drastically worse than anticipated, resulting in the drone’s jerky motion. However, we were able to improve the detection by printing a larger ArUco tag. Note that this will not be an issue in real-life as we will be using other more robust ways of localization.


## Results

<https://youtu.be/ETLLsu_NmuM> 


## Future Extensions

1. Obstacle avoidance while navigating along the path back with real-time RVIZ display of path planning
2. Add ability to switch which part of the body or object the drone is tracking
3. Add ability to recognize and label specific types of movements ie. walking, running, throwing


### Real World Implications

1. Having a separate body be able to both take video and track the route of a person would be very useful to adventurers that do not have GPS or need to navigate very precisely. Once a trail has been stored in memory, the drone can lead someone back along that trail accurately and at an acceptable pace. Basically the drone could be a remote selfie stick with navigation features


2. A tracking mechanism that follows a specific part of a person can be applied to all sorts of objects and/or features. Accurately following a specific feature and recording its path could be used in all sorts of applications that involve analyzing movement.


## References 

RVIZ

- <https://github.com/DavidB-CMU/rviz_tools_py>

YOLO

- <https://github.com/deepakcrk/yolov5-crowdhuman>
- <https://github.com/pranoyr/head-detection-using-yolo>

Motion Control:

- <https://github.com/DavidB-CMU/rviz_tools_py>

Localization:

- <https://github.com/DavidB-CMU/rviz_tools_py>
- <https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/> 

Driver:

- <https://github.com/cbenjamin23/BWSI_Student_Code/tree/main/catkin_ws/src/challenge_1>
