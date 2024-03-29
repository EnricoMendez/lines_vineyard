# lines_vineyard
ROS package for research of crop row detection in vineyards. Worked in ROS Noetic.

## Folders descriptions  

### Launch 

This folder contains the launch files of the package 

### Scripts 

This folder contains the ROS nodes. 

### Rviz_config 

This folder contains the rviz configurations needed for image monitoring. 

### Worlds 

This folder contains the gazeebo world used in the simulation. 

### Images

This folder contains the images used in this README


## Installation
To follow this guide is needed ROS Noetic to be already installed. 

Go to the src directory in your ROS workspace. Change "catkin_ws" on the command to the name of your workspace.

$ cd ~/catkin_ws/src

Copy this respository in the src directory. 

$ git https://github.com/EnricoMendez/lines_vineyard.git

Compile the workspace using catkin_make.

$ cd ..
$ catkin_make

## Run the code

### Setup
There are three ways to use the code. Using simulation, connect to a physical platform, and using a rosbag to simulate one of the test made by the owners of this repository.

#### Simulation

Download the simulation using the guide in the section of "Software used" subsection "Simulation" of this README.

Once you have the simulation running in Gazebo, add two boxes and make them into two large walls to the side of the Jackal. As shown in the next image:

![alt text](https://github.com/EnricoMendez/lines_vineyard/blob/main/images/imagen_2022-12-02_013723225.png?raw=true)

Then, open the navigation node located in the scripts folder.

$ cd ~/catkin_ws/src/lines_vineyard/scripts/
$ nano navigation_node.py

Comment line 24 and uncomment line 25 of the file and save the changes.

Now you can run the code.

$ cd ~/catkin_ws/
$ rosrun lines_vineyard navigation_node.py


#### Physical platform

To use this code in a physical platform is needed a robotic paltform and a camera connected to it. For this work we used the robot Jackal of Clearpath and connected a webcam model Logitech C525.

Install the Software indicated in the section of "Software used" subsection "Webcam" of this README. When is done you can turn on the camera to start navigating. The device can be different "/dev/video0" could be "/dev/video1" or "/dev/video2".

$ rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=yuyv  

Finally, run the ROS node.

$ cd ~/catkin_ws/
$ rosrun lines_vineyard navigation_node.py


#### Rosbag test

Using a rosbag allow other people to see the data collected during our tests. In the "Additional Material" section of this README is the link for some of rosbags that can be used to run the code.

Open a terminal and go to the "bags" directory

$ cd ~/catkin_ws/src/lines_vineyard/bags/

Play one of the rosbags in the folder

$ rosbag play [file_name.bag] -l

The -l make the rosbag to play in a loop. [file_name.bag] could be any of the files in the directory.

Finally, run the code.

$ cd ~/catkin_ws/
$ rosrun lines_vineyard navigation_node.py


## Visualition with Rviz

To know what is seeing the camera and what is happening with the segmetation use the rviz congifuration file available in the repository.

Open Rviz and click the File button

![alt text](https://github.com/EnricoMendez/lines_vineyard/blob/main/images/rviz_guide1.png?raw=true)

Select "Open Config". Go to the directory of this repository and open the rviz_config folder

![alt text](https://github.com/EnricoMendez/lines_vineyard/blob/main/images/rviz_guide2.png?raw=true)

Select the "visor.rviz" file

![alt text](https://github.com/EnricoMendez/lines_vineyard/blob/main/images/rviz_guide3.png?raw=true)

Now you should be able to see this view.

![alt text](https://github.com/EnricoMendez/lines_vineyard/blob/main/images/guide4.png?raw=true)

If you can't see it make sure you are running the navigation mode as explained above.


# Software used

This package was run using the following software
-Linux Ubunt 20.04 (Install here:)
-Ros Noetic (Install here: )

## Opencv

Opencv version 4.6.0.66 is used in this pacakge. 
To install use: pip install opencv-python
Open cv documentation can be found here: https://docs.opencv.org/4.x/index.html


## Webcam

First install the v4l-util Ubuntu package. It is a collection of command-line V4L utilities used by the usb_cam package: 

$ sudo apt-get install v4l-utils 

Then install the ROS usb_cam package: 

$ sudo apt-get install ros-$ROS_DISTRO-usb-cam  

To verify the PC recognize the camera:

$ dmesg

To connect the camera with the ROS initialize a roscore and execute:

$ rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=yuyv  

Parameter "/dev/video0" can change depending on where is the device detected. (/dev/video2 worked for us)

If the error "Unable to locate package ros-noetic-usb-cam" appear, clone the package to your workspace/src:

$ git clone https://github.com/bosch-ros-pkg/usb_cam.git

Finally, to visualize the video run:

$ rosrun image_view image_view image:=/usb_cam/image_raw 

## Simulation

Download the ROS worlds from clearpath: cd ~/catkin_ws/src
git clone https://github.com/clearpathrobotics/cpr_gazebo.git

To launch the world we used for testing run: 

roslaunch cpr_agriculture_gazebo agriculture_world.launch platform:=jackal


## Additional material  

In this drive folder ROS bags and videos from the experiments can be found in:  https://drive.google.com/drive/folders/1dIzErmy8LNG2bY9_CeFCYX4DfP9003LY?usp=sharing 
