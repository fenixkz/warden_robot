#!/usr/bin/env python
import rospy

# The name of the parameter to define the environment size.
# It should be a list `[max_x, max_y]` such that x:[0, `max_x`) and y:[0, `max_y`).
PARAM_ENVIRONMENT_SIZE = 'config/environment_size'

# The name of a boolean parameter to active random testing.
# If the value is `False` a keyboard-based interface will be used to produce stimulus 
# (i.e., speech, gesture and battery signals). Instead, random stimulus will be generate 
# if `True`. In the latter case, the architecture also requires all the parameters 
# with a the scope `test/random_sense/*`, which are not used if `False`.
PARAM_RANDOM_ACTIVE = 'test/random_sense/active'


# The name of parameter to set the initial robot position.
PARAM_INITIAL_POSE = 'state/initial_pose'


# ---------------------------------------------------------
# CONFIGURATION PARAMETERS FOR THE STATE MACHINE
PARAM_ONTOLOGY_NAME = "config/ontology_name"
PARAM_ONTOLOGY_PATH = "config/ontology_path"
PARAM_BATTERY_TIME_CHARGE = 'config/charge_time'
PARAM_WAIT_IN_LOCATION = 'config/waiting_time'
PARAM_CHARGING_STATION = 'config/charging_station'

# ---------------------------------------------------------


# The name of the node representing the shared knowledge required for this scenario.
NODE_ROBOT_STATE = 'robot-state'

# The name of the server to get the current robot pose.
SERVER_GET_POSE = 'state/get_pose'

# The name of the server to set the current robot pose. 
SERVER_SET_POSE = 'state/set_pose'

# The name of the topic where the battery state is published.
TOPIC_BATTERY_LOW = 'state/battery_low'

# The delay between changes of battery levels, i.e., high/low.
# It should be a list `[min_time, max_time]`, and the battery level change
# will occur after a random number of seconds within such an interval.
PARAM_BATTERY_TIME = 'test/random_sense/battery_time'
# ---------------------------------------------------------


# The name of the planner node.
NODE_PLANNER = 'planner'

# The name of the action server solving the motion planning problem.
ACTION_PLANNER = 'motion/planner'

# The number of points in the plan. It should be a list `[min_n, max_n]`,
# Where the number of points is a random value in the interval [`min_n`, `max_n`).
PARAM_PLANNER_POINTS = 'test/random_plan_points'

# The delay between the computation of the next via points.
# It should be a list `[min_time, max_time]`, and the computation will 
# last for a random number of seconds in such an interval.
PARAM_PLANNER_TIME = 'test/random_plan_time'
# -------------------------------------------------


# The name of the controller node.
NODE_CONTROLLER = 'controller'

# The name of the action server solving the motion control problem.
ACTION_CONTROLLER = 'motion/controller'

# The time required to reach a via points.
# It should be a list `[min_time, max_time]`, and the time to reach a
# via point will be a random number of seconds in such an interval.
PARAM_CONTROLLER_TIME = 'test/random_motion_time'
# -------------------------------------------------


# Function used to label each log with a producer tag.
def tag_log(msg, producer_tag):
    return f'@{producer_tag}>> {msg}'
