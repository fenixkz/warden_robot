# Inspection Robot 
My solution to Assignment I of Experimental Robotics class of the University of Genoa (UniGe). 

Author: Ayan Mazhitov.  
Main: mazhitovayan@gmail.com
## Introduction
This repository simulates the robot in a indoor environment for surveillance purposes. The main idea was to use [SMACH](http://wiki.ros.org/smach) library to create a State Machine that can simulate the needed behavior. Furthermore, the current repository uses [ARMOR](https://github.com/EmaroLab/armor) library and different scripts described in [arch_skeleton](https://github.com/buoncubi/arch_skeleton), it also creates an ontology and does some queries and manipulations to it, using [armor_py_api](https://github.com/EmaroLab/armor_py_api). 

The repository was implemented on Docker container with Ubuntu 20.04 with ROS Kinetic and Python 3.8.  
## Documentation
Documentation for the project can be found [here](https://fenixkz.github.io/warden_robot/)

## Scenario
The robot is used to inspect different locations in the indoor environment. The robot’s objective is to visit different locations and stay there for some times.

The surveillance pollicy is as folows:
 - When the robot's battery is not low, it should move to different locations and stay there for amount of time specified beforehand
 - The robot should stay mainly in the corridors
 - If a reachable room has not been visited for some times it should visit it.
 - If the robot's battery got low, it should go to the location containing the charging station and recharge the battery.
## Installation and launching
### Requirements
The machine has to have ROS Noetic with python > 3.0. Also, the following libraries have to be installed beforehand
 - [rospy](http://wiki.ros.org/rospy)
 - [actionlib](http://wiki.ros.org/actionlib)
 - [ARMOR](https://github.com/EmaroLab/armor)
 - [armor_py_api](https://github.com/EmaroLab/armor_py_api)
 - xterm `sudo apt install -y xterm`
 - [smach](http://wiki.ros.org/smach)
 
### Installation 
 - Clone the current repository to your ROS workspace 
 - Change directory to _scripts_ and run `chmod +x *.py` where * is the name of each python script
 - Finally, build your workspace

### Launching
Before launching the solution, please open the `warden_robot/launch/main.launch` file and change the path to the ontology file in `config/ontology_path` parameter.  
To launch the solution, please source your workspace and run the following command:  
`roslaunch warden_robot main.launch random:=false view_smach_gui:=false`  
Two arguments are given to the launch file.  
`random` refers to either random sense or manual sense of controlling the battery of the robot.  
`view_smach_gui` arguments responsible for visualizing the state machine
### Parameters
In the `main.launch` file you can find the parameters used for the state machine implementation:
 - `test/random_sense/active` takes the argument `random` and decides whether the state machine is going to use random or manual sense
 - `test/random_plan_points` is a list of two integer numbers representing the amount of via points given by the **Planner** action server
 - `test/random_plan_time` is a list of two float numbers representing a delay to simulate computation of via points
 - `test/random_motion_time` is a list of two float numbers representing a delay to simulate the time spent moving the robot to that location
 - `state/initial_pose` is a list of two float numbers representing the initial position of the robot in the environment
 - `config/waiting_time` is a float number representing the waiting time of the robot in a location for surveillance purposes
 - `config/charge_time` is a float number representing the waiting time to simulate a process of recharging the battery of the robot
 - `config/environment_size` is a list of two float numbers representing the size of the environment
 - `config/charging_station` is a string representing the location where the charging station is located
 - `config/ontology_name` is a string representing the name of the ontology
 - `config/ontology_path` is a string representing the absolute path to the ontology file
 - `test/random_sense/battery_time` is a list of two float numbers for choosing randomly the amount of time to wait to change the state of the battery. Only needed when the random sense is active. 
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
The program starts with the Phase 1. This phase includes only one state: **`BUILD_MAP`** which has only two possible output: `MAP_HAS_BUILT` and `MAP_HAS_NOT_BUILT`. The latter output transits to itself, while the former one transits to the **`START_EXPLORING`** state and starts the normal behavior of the robot.

### Phase 2
Phase 2 has some hierarchical states. There are two higher level states: **`START_EXPLORING`** and **`START_CHARGING_ROUTINE`**. 

#### Start Exploring State
This high level state incorporates four lower-level states and has two outputs. It is used as the main state to do the normal surveillance of the robot. 

The possible outputs are: `REPEAT` and `RECHARGING_THE_BATTERY`. The first one transits the state to itself to do the inspection in a infinite loop, and the second one refers to the transition when the battery got low and transits the state machine to **`START_CHARGING_ROUTINE`** state.  

Four inner states are: **`START_BEHAVIOR`**, **`PLAN_TO_GIVEN_LOCATION`**, **`GO_TO_GIVEN_LOCATION`**, and **`WAIT_IN_LOCATION`**.   
**`START_BEHAVIOR`** is the initial state of the outer state, in this state the program decides the next location, where the robot should go and transits to **`PLAN_TO_GIVEN_LOCATION`**. In that state, the algorithm calls the Planner Action server to compute the plan to the desired location. After the planner is done, it transits to **`GO_TO_GIVEN_LOCATION`**, where the algorithm calls the Controller Action server to follow the computed plan and moves the robot to the new location.  
When the robot has successfully moved to a new location, the state machine goes to the final state **`WAIT_IN_LOCATION`**, where the robot waits for the specified amount of time.  
Every of that states has `RECHARGING_THE_BATTERY` transitions. That transitions is executed when the robot's battery got low while executing the state routine.

#### Start Charging Routine State
This high level state has three inner states and one possible output - `BATTERY_IS_FULL`, which trasmits the state machine to **`START_EXPLORING`** state.  

The initial state is **`PLAN_TO_CHARGING_LOCATION`**, where the algorithm checks whether the robot is in location containing the charging station. If yes, then it transits to **`RECHARGE`** state, where the algorithm simulates the recharging process of the battery. If no, then it finds the location closest to the charging station and computes a plan to it.  
After that, it transits to **`GO_TO_CHARGING_LOCATION`** state, where the Controller Action server follows the computed plan and checks again whether the robot is within the charging location. If yes, then it transits to **`RECHARGE`** and if no, then the routine repeats.  

Finally, **`RECHARGE`** state check if the battery got full. If the battery is full, then it transits to `BATTERY_FULL`, if not it transits to itself and does it until the battery gets full.

## Software architecture
The _scripts/_ folder contains the information on each software component that was used to create this repository.  
**Planner.py** and **Controller.py** were given to us and more in-depth explanation of these software can be found [here](https://github.com/buoncubi/arch_skeleton)
### State_helper
**state_helper.py** implements helper classes to deal with `armor_py_api` and overall logic of the surveillance policy. It incorporates three classes: _ProtegeHelper()_, _ActionClientHelper()_, and _InterfaceHelper()_. The documentation provides the explanation of their logic.
### State_machine
This is the main python module that creates the SMACH like State Machine and does the solution to the assignment. Again, please refer to the documentation for more detailed explanation.

## Problems and further improvements
The solution works very well for the provided as default parameters. However, some problems may occur when you change the location where the charging station is located. With this connection further improvements are:
 - Improve the `state_helper.ProtegeHelper.plan_to_recharge_station` method to work for other charging locations
 - Use the previously computed plans to move between the same locations insted of computing the new one for better performance
 - Implement the possibility of adding and controlling more than one robot
 - Implement the behavior when either **Planner**, **Controller** or **armor_py_api** fails
