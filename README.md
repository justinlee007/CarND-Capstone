# System Integration Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This is the final project for the Udacity Self-Driving Car Engineer Nanodegree.  In this project, our team created several ROS nodes to implement core functionality of an autonomous vehicle.  For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

[//]: # (Image References)
[image1]: ./imgs/rosgraph.jpg

## Team Members
The members of team <b>MunixX</b>:

| Name                          | Slack handle | GitHub account                                    | Udacity Email                       |
|:------------------------------|:-------------|:--------------------------------------------------|-------------------------------------|
| Clemens Habedank (team lead)  | @clemens     | [CleWiDank](https://github.com/CleWiDank)         | clemens.w.habedank@gmail.com        |
| Justin Lee                    | @justinlee   | [justinlee007]( https://github.com/justinlee007)  | justin.lee007@gmail.com             |
| Aaron Piper                   | @apiper0770  | [apiper0770]( https://github.com/apiper0770)      | apiper0770@gmail.com                |
| Qiong Gui                     | @guiq        | [guiqiong](https://github.com/guiqiong)           | guiqcn@gmail.com                    |
| Tan Wang                      | @timn9408    | [dayuwater](https://github.com/dayuwater)         | timmy940408@hotmail.com             |

## Setup

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

## Project Overview

### Carla Architecture
Carla is the custom Lincoln MKZ that Udacity has converted into a self-driving car.  It's self-driving system is broken down into for major sub-systems: *Sensors*, *Perception*, *Planning* and *Control* 

#### Sensors
Includes everything needed to understand its surroundings and location

* Cameras
* Lidar
* GPS
* Radar
* IMU

#### Perception
Abstracts sensor inputs into object detection and localization
* Detection
* Localization
#### Planning
##### Route Planning
The route planning component is responsible for high-level decisions about the path of the vehicle between two points on a map; for example which roads, highways, or freeways to take. This component is similar to the route planning feature found on many smartphones or modern car navigation systems.
##### Prediction
The prediction component estimates what actions other objects might take in the future. For example, if another vehicle were identified, the prediction component would estimate its future trajectory.
##### Behavioral Planning
The behavioral planning component determines what behavior the vehicle should exhibit at any point in time. For example stopping at a traffic light or intersection, changing lanes, accelerating, or making a left turn onto a new street are all maneuvers that may be issued by this component.
##### Trajectory Planning
Based on the desired immediate behavior, the trajectory planning component will determine which trajectory is best for executing this behavior.
### Control
The control component takes




### ROS Architecture

The ROS Architecture consists of different nodes (written in Python or C++) that communicate with each other via ROS messages. The nodes and their communication with each other are depicted in the picture below. The ovally outlined text boxes inside rectangular boxes represent the ROS nodes while the simple rectangular boxes represent the topics that are subscribed or published to. The direction of the arrows clarifies the respective flow of communication. 

![][image1]

The most central point in the rqt-graph is the styx_server that links the simulator and ROS by providing information about the car's state and surroundings (car's current position, velocity and images of the front camera) and receiving control input (steering, braking, throttle). The other nodes can be associated with the three central tasks Perception, Planning and Control. 

The images get processed within the traffic light classifier by a trained neural network in order to detect traffic lights. The percepted state of a potentially upcoming traffic light is passed to the traffic light detector as well as the car's current pose and a set of base waypoints coming from the waypoint loader. With this frequently incoming information the traffic light detector is able to publish a waypoint close to the next traffic light where the car should stop in case the light is red. 

With the subscribed information of the traffic light detector and the the subscriptions to base waypoints, the waypoint updater node is able to plan acceleration / deceleration and publish it to the waypoint follower node. This node publishes to the DBW (Drive by wire) node that satisfies the task of steering the car autonomously. It also takes as input the car's current velocity (coming directly from the car / simulator) and outputs steering, braking and throttle commands. 

### Node Design

In this paragraph it will be talked about the node design of those nodes that are built within this project. Those are the waypoint updater(waypoint_updater.py), the traffic light detector (tl_detector.py) and the drive by wire node (dbw_node.py). 

The waypoint updater node takes a central role in the planning task because it determines which waypoints the car should follow. The node is structured into different parts: First an import-part, where some python libraries and some message formats are imported, followed by the initialisation of some constants that are not supposed to be changed, e.g. how many waypoints are published and in which rate. After this part the class WaypointUpdater is introduced. It is is structured into different functions. The first function is the init-function defining the attributes of the class and determining which topics the class subscribes to and which ones it publishes. 
The following functions are either general methods or callback functions that are invoked repeatedly by the subscribers in the init-function. Repeatedly called are the base waypoints (output of waypoint loader), the car's pose (simulator / car) and the traffic waypoint (output of tl_detector). The most important general method is the decelerate_waypoints-function which incorporates a square-root shaped deceleration towards a stopline in case the traffic light is red.  
At the end of the node there is the main function that runs the node and logs an error in case ROS is interrupted by some reason. 

The structure of the traffic light detector is the same in the way that there is an import/initialisation-part followed by a class with attributes and functions and the main functions that compiles the code. The init-function of the TLDetector class includes the subscriptions to the current position, the base waypoints, the given traffic light array with the coordinates of the traffic lights and the color of the traffic light. The color of the traffic light is the output of the traffic light classifier, a neural network that is explained more in detail in the next paragraph. The topic image_color gets updated by the callback image_cb which itself calls via process_traffic_lights() the function get_light_state() that receives the traffic light classification. Eventually, the waypoint of a potentially upcoming red traffic light is published.

The third node written by us is the dbw_node which is responsible for steering the car. It subscribes to a twist controller which outputs throttle, brake and steering values with the help of a PID-controller and Lowpass filter. The dbw node directly publishes throttle, brake and steering commands for the car /simulator, in case dbw_enabled is set to true. 

### Neural Network Design

## Testing

## Results
