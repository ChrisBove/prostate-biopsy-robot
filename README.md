# prostate-biopsy-robot
Modeling and Control of Needle-Guiding Prostate Biopsy Robot - RBE 502 Final Project 

## Contents
- matlab: all matlab code for testing simulations and needle modeling
- pbb_visualization: ROS package for managing interactive markers and componenets for visualization of the system in RViz.

## Workspace Setup
You need Ubuntu 14.04 with ROS indigo installed. Within your catkin workspace, clone this repository and run `catkin_make` in the root of your catkin workspace.

## Running 
```
roslaunch pbb_visualization visualize_bringup.launch
```
