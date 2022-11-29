# Inspection Robot 
My solution to Assignment I of Experimental Robotics class of the University of Genoa (UniGe)
## Introduction
This repository simulates the robot in a indoor environment for surveillance purposes. The main idea was to use [SMACH](http://wiki.ros.org/smach) library to create a State Machine that can simulate the needed behavior. Furthermore, the current repository uses [ARMOR](https://github.com/EmaroLab/armor) library and different scripts described in [arch_skeleton](https://github.com/buoncubi/arch_skeleton), it also creates an ontology and does some queries and manipulations to it, using [armor_py_api](https://github.com/EmaroLab/armor_py_api). 

The repository was implemented on Docker container with Ubuntu 20.04 with ROS Kinetic and Python 3.8.  
## Documentation
Documentation for the project cana be found [here](https://fenixkz.github.io/warden_robot/)

## Scenario
The robot is used to inspect different locations in the indoor environment. The robot’s objective is to visit different locations and stay there for some times.

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

The assignment implies several assumptions:
 * Planner and controller routines are already implemented by the professor and can be found in the scripts folder
 * These are dummy implementation and more in depth description of these services can be found [here](https://github.com/buoncubi/arch_skeleton)
 * The planner requires **x** and **y** coordinates to compute the plan. Thus, I attached coordinates of the center of each location as shown in the environment figure
 * The location becomes urgent when the last timestamp of that location was more *urgencyThreshold* seconds ago 

## State Diagram
The state machine is divided into two phases. 
1. First phase represents the following behavior: The robot starts in the Location E and waits until it receives all the information about the map from _armor_py_api_
2. Second phase represents the normal behavior described by the surveillance pollicy shown above

The diagram of the states is shown below. Each of the phase is indicated in the figure. 
![state_machine](https://github.com/fenixkz/warden_robot/blob/main/figures/state_diagram.jpg)

## States and transitions
### Phase 1
The program starts with the Phase 1. This phase includes only one state: **`BUILD_MAP`** which has only two possible output: **`MAP_HAS_BUILT`** and **`MAP_HAS_NOT_BUILT`**. The latter output transits to itself, while the former one transits to the **`START_EXPLORING`** state.

### Phase 2
Phase 2 has some hierarchical states. There are two higher level states: **START_EXPLORING** and **START_CHARGING_ROUTINE**. 

#### Start Exploring State
This high level state incorporates four lower-level states and has two outputs. 

