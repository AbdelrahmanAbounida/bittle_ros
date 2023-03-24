# Robust Fiducial Markers Detection And Following Using Quadruped Robot (ROS)

[![Made with ROS](https://img.shields.io/badge/Made%20with-ROS-green?&logo=ros)](http://wiki.ros.org/)
[![Python 3.8](https://img.shields.io/badge/Python-3.8-3776AB?logo=python)](https://www.python.org/downloads/release/python-360/)
[![Gazebo](https://img.shields.io/badge/GAZEBO-orange?logo=gazebo&logoColor=white)](https://gazebosim.org/home)

<p align="center">
<img  src="./follow.gif" />
</p>

<br />
<br />
<br />

![](./assets/main_controller.png)

# Project Structure

This ROS Environment consists of 4 packages:
- ## 1- bittle_description: 
This package includes bittle urdf model
<br />

- ## 2- bittle_gazebo :
This package used for spawning bittle robot
<br />

- ## 3- apriltag_pose_estimation:
This package is used to estimate and publish the current distance between camera and Apriltag
<br />

- ## 4- apriltag_follower: 
the main package in which Apriltag detection process is implemented
<br />


 Bittle Model             |   Apriltags
:-------------------------:|:-------------------------:
![](https://hackster.imgix.net/uploads/attachments/1350269/hackster-front_O66b4x4vua.gif?auto=format%2Ccompress&gifq=35&w=900&h=675&fit=min&fm=mp4)  |  ![](https://cdn.shopify.com/s/files/1/0292/0693/7678/files/apriltag-pad_1_grande.png?v=1594511445)


# Getting Started


# 1- Simulation (simulation) ##

> **_NOTE:_** <br />
1- This Robot model urdf is cloned from this repo, [Bittle_URDF](https://github.com/AIWintermuteAI/Bittle_URDF) <br /> 2- while the JointEffortService and the mixer implemenation has been cloned from this repo  [notspot_sim_py](https://github.com/lnotspotl/notspot_sim_py)

<p align="center">
<img  src="./assets/follow.gif" />
</p>

1. Clone this project. 
    - 
    ~~~bash
    $ cd construction-robotics-ws-2022_23/simulation
    ~~~
2. Building code in a catkin workspace and adding the packages to current running local ros. 
    - 
    ~~~bash
    $ catkin_make
    ~~~
    ~~~bash
    $ source devel/setup.bash
    ~~~
3. Open the simulation environment.
    - 
    ~~~bash
    $ roslaunch bittle_gazebo simulation.launch
    ~~~
4. Spawn Bittle Model (in new terminal).
    - 
    ~~~bash
    $ source devel/setup.bash
    ~~~
    ~~~bash
    $ roslaunch bittle_gazebo spawn_robot.launch
    ~~~
5. Estimate the camera distance to the apriltag (in new terminal).
    -
    ~~~bash
    $ source devel/setup.bash
    ~~~
    ~~~bash
    $ roslaunch apriltag_pose_estimation apriltag_detector.launch
    ~~~
6. Follow apriltag (in new terminal).
    - 
    ~~~bash
    $ source devel/setup.bash
    ~~~
    ~~~bash
    $ roslaunch apriltag_follower apriltag_follower.launch
    ~~~




<div align="center"> :star: :star: :star:  </div>


