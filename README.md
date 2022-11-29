# Inspection Robot 
My solution to Assignment I of Experimental Robotics class of the University of Genoa (UniGe)
## Introduction
This repository simulates the robot in a indoor environment for surveillance purposes. The main idea was to use [SMACH](http://wiki.ros.org/smach) library to create a State Machine that can simulate the needed behavior. Furthermore, the current repository uses [ARMOR](https://github.com/EmaroLab/armor) library and different scripts described in [arch_skeleton](https://github.com/buoncubi/arch_skeleton), it also creates an ontology and does some queries and manipulations to it, using [armor_py_api](https://github.com/EmaroLab/armor_py_api). 

The repository was implemented on Docker container with Ubuntu 20.04 with ROS Kinetic and Python 3.8.  
Further, the documentation for the project can be found [here](https://github.com/fenixkz)
## Scenario
The robot is used to inspect different locations in the indoor environment. The robot’s objective is to visit the different locations and stay there for some times.

The surveillance pollicy is as folows:
 - When the robot's battery is not low, it should move to different locations 
 - The robot should stay mainly in the corridors
 - If a reachable room has not been visited for some times it should visit it.
 - If the robot's battery got low, it should go to the location containing the charging station and recharge the battery.

## Environment
The robot is simulated in the indoor environment which is shown in the figure below 
![env](https://github.com/fenixkz/warden_robot/blob/main/figures/exprorob.png)

The environment consists of locations and doors, specifically:
 - A room is a location with one door
 - A corridor is a location with more than two doors
 - If two locations share the same door, the robot can move between these locations

The skeleton of the environment has been created by __Luca Buoncomapgni__ [here](https://github.com/buoncubi/topological_map), which can be found in the _topological_map_ folder. 

## State Diagram


