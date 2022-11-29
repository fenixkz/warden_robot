#!/usr/bin/env python

import rospy
import smach_ros
from smach import StateMachine, State
import math

from warden_robot import architecture_name_mapper as anm
from warden_robot.msg import Point, PlanGoal, ControlGoal
from state_helper import ProtegeHelper, InterfaceHelper

# Define states of the state machine

STATE_PLAN_TO_LOCATION = 'PLAN_TO_GIVEN_LOCATION'
STATE_GO_TO_LOCATION = 'GO_TO_GIVEN_LOCATION'
STATE_BUILD_MAP = "BUILD_MAP" # State when the robot waits until the topological map is built
STATE_START_EXPLORING = "START_EXPLORING" # Start the normal behavior of phase 2
STATE_START_BEHAVIOR = "START_BEHAVIOR"
STATE_RECHARGE = "RECHARGE"
STATE_WAIT_IN_LOCATION = "WAIT_IN_LOCATION"

STATE_GO_TO_CHARGING_LOCATION = "GO_TO_CHARGING_LOCATION"
STATE_PLAN_TO_CHARGING_LOCATION = "PLAN_TO_CHARGING_LOCATION"
STATE_RECHARGING_ROUTINE = "START_RECHARGING_ROUTINE"

# Define transitions
TRANS_MAP_NOT_BUILT = "MAP_HAS_NOT_BUILT" # Map is not have yet been built
TRANS_MAP_BUILT = "MAP_BUILT" # Map has been built 
TRANS_REPEAT = "REPEAT"
TRANS_BATTERY_LOW = "BATTERY_IS_LOW"
TRANS_BATTERY_FULL = "BATTERY_IS_FULL"
TRANS_PLAN_TO_LOCATION = "PLAN_TO_LOCATION"
TRANS_PLANNED_TO_LOCATION = "VIA_POINTS_HAVE_BEEN_COMPUTED"
TRANS_GO_TO_LOCATION = "GO_TO_LOCATION"
TRANS_WENT_TO_LOCATION = "ROBOT_HAS_MOVED_TO_GIVEN_LOCATION"
TRANS_RECHARGING = "RECHARGING_THE_BATTERY"
TRANS_DONE_WAITING = "DONE_WAITING"

TRANS_IN_CHARGING_LOCATION = "ROBOT_IS_IN_CHARGING_LOCATION"
TRANS_NOT_IN_CHARGING_LOCATION = "ROBOT_IS_NOT_IN_CHARGING_LOCATION"



LOOP_SLEEP_TIME = 0.1

LOG_TAG = "STATE_MACHINE"

class BuildMap(State):
	def __init__(self, protegeHelper):
		# Protege Helper class that helps with dealing with armor_py_api client
		self.helper = protegeHelper
		State.__init__(self, outcomes=[TRANS_MAP_BUILT, TRANS_MAP_NOT_BUILT])
	def execute(self, userdata):
		rospy.loginfo(anm.tag_log("Building the map, please wait ...", LOG_TAG + " >>> " + STATE_BUILD_MAP))
		if self.helper.build_map():
			rospy.loginfo(anm.tag_log("Great. Map has been built.", LOG_TAG + " >>> " + STATE_BUILD_MAP))
			rospy.loginfo("Okay. \n List of rooms = {0} \n List of corridors = {1} \n List of doors = {2}".format(self.helper.get_rooms_list(), self.helper.get_corridors_list(), self.helper.get_doors_list()))
			t0 = math.floor(rospy.Time.now().to_sec())
			self.helper.move_robot('E', t0)
			log = "The robot is now in location {0} at time {1} nsec".format('E', t0)
			rospy.loginfo(anm.tag_log(log, LOG_TAG + " >>> " + STATE_BUILD_MAP))
			return TRANS_MAP_BUILT
		else:
			rospy.sleep(10)
			rospy.logerr("Unexpected error when trying to build the map. Trying again..")
			return TRANS_MAP_NOT_BUILT

class StartBehavior(State):
	def __init__(self, protegeHelper, interfaceHelper):
		State.__init__(self, outcomes=[TRANS_REPEAT, TRANS_PLAN_TO_LOCATION], output_keys = ['location'])
		self._pHelper = protegeHelper
		self._iHelper = interfaceHelper
	def execute(self, userdata):
		# Get from Protege the list of all reachable locations
		reachable_locations = self._pHelper.canReach()
		if reachable_locations: # Check if the returned list is not empty
			userdata.location = self._pHelper.decide_next_location()
			return TRANS_PLAN_TO_LOCATION
		else:
			rospy.sleep(3)
			rospy.logerr("Cannot obtain the list of reachable locations from armor_py_api")
			return TRANS_REPEAT

"""
Class to plan a path to charging station
"""
class PlanToChargingStation(State):
	def __init__(self, interface_helper, protege_helper):
		self._iHelper = interface_helper
		self._pHelper = protege_helper
		State.__init__(self, outcomes = [TRANS_BATTERY_FULL, TRANS_IN_CHARGING_LOCATION, TRANS_GO_TO_LOCATION], input_keys = [], output_keys = ['t0', 'via_points', 'room'])
	
	def execute(self, userdata):
		goal = PlanGoal()
		charging_location = self._iHelper.get_charging_location()
		if self._pHelper.get_current_location() == charging_location: # If the robot is already in the location containing the charging station, then go to recharge state
			userdata.t0 = rospy.Time.now().to_sec()
			return TRANS_IN_CHARGING_LOCATION
		else: # If not, then plan the path to reach that location
			next_location = self._pHelper.plan_to_recharge_station(charging_location)
			dict = self._iHelper.get_coord_dict()
			goal.target = Point(x=dict[next_location][0], y=dict[next_location][1])
			self._iHelper.planner_client.send_goal(goal)
			log = "Robot has low battery. Need to go to charging station. Therefore, planning to go in location {0}".format(next_location)
			rospy.loginfo(anm.tag_log(log, LOG_TAG + " >>> " + STATE_PLAN_TO_CHARGING_LOCATION))
			while not rospy.is_shutdown():
				self._iHelper.mutex.acquire()
				try:
					if not self._iHelper.is_battery_low(): # If somehow the battery go recharged ( Not possible in the real life scenario, however, to make random sense work )
						rospy.loginfo(anm.tag_log("Miracle. Robot's battery got full...", LOG_TAG + " >>> " + STATE_PLAN_TO_CHARGING_LOCATION))
						self._iHelper.planner_client.cancel_goals()
						return TRANS_BATTERY_FULL
					if self._iHelper.planner_client.is_done():
						userdata.via_points = self._iHelper.planner_client.get_results().via_points
						userdata.room = next_location
						return TRANS_GO_TO_LOCATION
				finally:
					self._iHelper.mutex.release()
				rospy.sleep(LOOP_SLEEP_TIME)
				
				
class GoToChargingStation(State):
	def __init__(self, interface_helper, protege_helper):
		self._iHelper = interface_helper
		self._pHelper = protege_helper
		State.__init__(self, outcomes = [TRANS_BATTERY_FULL, TRANS_IN_CHARGING_LOCATION, TRANS_NOT_IN_CHARGING_LOCATION], input_keys = ['via_points', 'room'], output_keys = ['t0'])
	
	def execute(self, userdata):
		
		goal = ControlGoal(via_points=userdata.via_points)
		charging_location = self._iHelper.get_charging_location()
		self._iHelper.controller_client.send_goal(goal)
		log = "Robot has low battery. Need to go to charging station. Therefore, following the plan to reach a room {0} ...".format(userdata.room)
		rospy.loginfo(anm.tag_log(log, LOG_TAG + " >>> " + STATE_GO_TO_CHARGING_LOCATION))
		# Wait for the action server computation and listen possible incoming stimulus.
		while not rospy.is_shutdown():
			# Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
			self._iHelper.mutex.acquire()
			try:
				if not self._iHelper.is_battery_low(): # If somehow the battery go recharged ( Not possible in the real life scenario, however, to make random sense work )
					rospy.loginfo(anm.tag_log("Miracle. Robot's battery got full...", LOG_TAG + " >>> " + STATE_GO_TO_CHARGING_LOCATION))
					self._iHelper.controller_client.cancel_goals()
					return TRANS_BATTERY_FULL
				if self._iHelper.controller_client.is_done():
					t = math.floor(rospy.Time.now().to_sec())
					self._pHelper.move_robot(userdata.room, t)
					log = "The robot is now in location {0}".format(userdata.room)
					rospy.loginfo(anm.tag_log(log, LOG_TAG + " >>> " + STATE_GO_TO_CHARGING_LOCATION))
					if self._pHelper.get_current_location() == charging_location:
						userdata.t0 = rospy.Time.now().to_sec()
						return TRANS_IN_CHARGING_LOCATION
					else:
						return TRANS_NOT_IN_CHARGING_LOCATION
			finally:
				# Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
				self._iHelper.mutex.release()
			# Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
			rospy.sleep(LOOP_SLEEP_TIME)

class Recharge(State):
	def __init__(self, interface_helper):
		State.__init__(self, outcomes=[TRANS_BATTERY_FULL, TRANS_BATTERY_LOW], input_keys = ['t0'])
		self._iHelper = interface_helper
		self.charging_time = self._iHelper.charging_time
	def execute(self, userdata):
		rospy.sleep(0.1)
		if not self._iHelper.randomness: # If the sense is manual, then simulate like the robot is charging; 
			if self.non_blocking_wait(userdata.t0): # Simulate charging the battery
				self._iHelper.reset_battery() # Reset the battery to say that is is not low anymore
		else: # If the sense is random, then just wait until the battery topic gets informed that the battery is full
			rospy.loginfo(anm.tag_log("Waiting until the battery gets full...", LOG_TAG + " >>> " + STATE_RECHARGE))
		if not self._iHelper.is_battery_low():
			return TRANS_BATTERY_FULL
		else:
			return TRANS_BATTERY_LOW
	def non_blocking_wait(self, t0):
		percent = (rospy.Time.now().to_sec() - t0) / self.charging_time * 100
		log = "Battery is {0}%".format(percent)
		rospy.loginfo(anm.tag_log(log, LOG_TAG + " >>> " + STATE_RECHARGE))
		return t0 + self.charging_time < rospy.Time.now().to_sec()


class PlanToLocation(State):
	def __init__(self, interface_helper):
		# Get a reference to the interfaces with the other nodes of the architecture.
		self._iHelper = interface_helper
		# Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
		# Also, set the `random_plan` variable, which will be eventually computed in the `execute` function and passed to the `GO_TO_RANDOM_POSE` state.
		State.__init__(self, outcomes=[TRANS_RECHARGING, TRANS_PLANNED_TO_LOCATION], input_keys=['location'], output_keys=['via_points', 'room'])

	# Define the function performed each time a transition is such to enter in this state.
	def execute(self, userdata):

		goal = PlanGoal()
		# Get the dictionary to retrieve the coordinates that need to be given to the planner
		dict = self._iHelper.get_coord_dict()
		goal.target = Point(x=dict[userdata.location][0], y=dict[userdata.location][1])
		# Invoke the planner action server.
		self._iHelper.planner_client.send_goal(goal)
		log = "Planning to go in a location {0}".format(userdata.location)
		rospy.loginfo(anm.tag_log(log, LOG_TAG + " >>> " + STATE_PLAN_TO_LOCATION))
		# Wait for the action server computation and listen possible incoming stimulus.
		while not rospy.is_shutdown():
			# Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
			self._iHelper.mutex.acquire()
			try:
				# If the battery is low, then cancel the planning action server and take the `battery_low` transition.
				if self._iHelper.is_battery_low():  # Higher priority.
					rospy.loginfo(anm.tag_log("Robot's battery got low..", LOG_TAG + " >>> " + STATE_PLAN_TO_LOCATION))
					self._iHelper.planner_client.cancel_goals()
					return TRANS_RECHARGING

				if self._iHelper.planner_client.is_done():
					userdata.via_points = self._iHelper.planner_client.get_results().via_points
					userdata.room = userdata.location
					return TRANS_PLANNED_TO_LOCATION
			finally:
				# Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
				self._iHelper.mutex.release()
			# Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
			rospy.sleep(LOOP_SLEEP_TIME)


# The definition of the inner state of `NORMAL` concerning motion controlling to reach a random position.
class GoToLocation(State):
	# Construct this class, i.e., initialise this state.
	def __init__(self, protege_helper, interface_helper):
		# Get a reference to the interfaces with the other nodes of the architecture.
		self._iHelper = interface_helper
		self._pHelper = protege_helper
		# Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
		# Also, get the `random_plan` variable, which is computed by the `PLAN_TO_RANDOM_POSE` state.
		State.__init__(self, outcomes=[TRANS_WENT_TO_LOCATION, TRANS_RECHARGING], input_keys=['via_points', 'room'], output_keys=[])

	# Define the function performed each time a transition is such to enter in this state.
	def execute(self, userdata):
		# Start the action server for moving the robot through the planned via-points.
		goal = ControlGoal(via_points=userdata.via_points)
		self._iHelper.controller_client.send_goal(goal)
		log = "Following the plan to reach a room {0} ...".format(userdata.room)
		rospy.loginfo(anm.tag_log(log, LOG_TAG + " >>> " + STATE_GO_TO_LOCATION))
		# Wait for the action server computation and listen possible incoming stimulus.
		while not rospy.is_shutdown():
			# Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
			self._iHelper.mutex.acquire()
			try:
				# If the battery is low, then cancel the control action server and take the `battery_low` transition.
				if self._iHelper.is_battery_low():  # Higher priority
					self._iHelper.controller_client.cancel_goals()
					return TRANS_RECHARGING
				# If the controller finishes its computation, then take the `went_random_pose` transition, which is related to the `repeat` transition.
				if self._iHelper.controller_client.is_done():
					t = math.floor(rospy.Time.now().to_sec())
					self._pHelper.move_robot(userdata.room, t)
					log = "The robot is now in location {0}".format(userdata.room)
					rospy.loginfo(anm.tag_log(log, LOG_TAG + " >>> " + STATE_GO_TO_LOCATION))
					return TRANS_WENT_TO_LOCATION
			finally:
				# Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
				self._iHelper.mutex.release()
		# Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
		rospy.sleep(LOOP_SLEEP_TIME)

class WaitInLocation(State):
	def __init__(self, interface_helper):
		State.__init__(self, outcomes=[TRANS_DONE_WAITING, TRANS_RECHARGING], output_keys = ['t0'])
		self._iHelper = interface_helper
	def execute(self, userdata):
		rospy.loginfo(anm.tag_log("Waiting in the location", LOG_TAG + " >>> " + STATE_WAIT_IN_LOCATION))
		i = 0
		while i < self._iHelper.waiting_time:
			rospy.sleep(0.05) # 20 Hz, fast enough to check the battery meanwhile
			self._iHelper.mutex.acquire()
			try:
				# If the battery is low, then cancel the control action server and take the `battery_low` transition.
				if self._iHelper.is_battery_low():  # Higher priority
					userdata.t0 = rospy.Time.now().to_sec()
					return TRANS_RECHARGING
			finally:
				self._iHelper.mutex.release()
			i = i + 0.05
		rospy.loginfo(anm.tag_log("Done waiting", LOG_TAG + " >>> " + STATE_WAIT_IN_LOCATION))
		return TRANS_DONE_WAITING

def main():
	rospy.init_node("state_machine", log_level=rospy.INFO)
	sm_main = StateMachine(outcomes=[])
	protegeHelper = ProtegeHelper()
	interfaceHelper = InterfaceHelper()
	robot_pose_param = rospy.get_param(anm.PARAM_INITIAL_POSE, [0, 0])
	
	interfaceHelper.init_robot_pose(Point(x=robot_pose_param[0], y=robot_pose_param[1]))


	with sm_main:
		StateMachine.add(STATE_BUILD_MAP, BuildMap(protegeHelper), transitions={TRANS_MAP_NOT_BUILT: STATE_BUILD_MAP, TRANS_MAP_BUILT: STATE_START_EXPLORING})
		sm_behavior = StateMachine(outcomes=[TRANS_REPEAT, TRANS_RECHARGING])
		with sm_behavior:
			StateMachine.add(STATE_START_BEHAVIOR, StartBehavior(protegeHelper, interfaceHelper), transitions={TRANS_PLAN_TO_LOCATION: STATE_PLAN_TO_LOCATION, TRANS_REPEAT: TRANS_REPEAT})
			
			StateMachine.add(STATE_PLAN_TO_LOCATION, PlanToLocation(interfaceHelper), transitions= {TRANS_RECHARGING: TRANS_RECHARGING, TRANS_PLANNED_TO_LOCATION: STATE_GO_TO_LOCATION})
			
			StateMachine.add(STATE_GO_TO_LOCATION, GoToLocation(protegeHelper, interfaceHelper), transitions = {TRANS_RECHARGING: TRANS_RECHARGING, TRANS_WENT_TO_LOCATION: STATE_WAIT_IN_LOCATION})
			
			StateMachine.add(STATE_WAIT_IN_LOCATION, WaitInLocation(interfaceHelper), transitions = {TRANS_DONE_WAITING: TRANS_REPEAT, TRANS_RECHARGING: TRANS_RECHARGING})
			
			
			
		StateMachine.add(STATE_START_EXPLORING, sm_behavior, transitions={TRANS_REPEAT: STATE_START_EXPLORING, TRANS_RECHARGING: STATE_RECHARGING_ROUTINE})
		
		sm_recharge = StateMachine(outcomes = [TRANS_BATTERY_FULL])
		with sm_recharge:
			StateMachine.add(STATE_PLAN_TO_CHARGING_LOCATION, PlanToChargingStation(interfaceHelper, protegeHelper), transitions={TRANS_BATTERY_FULL: TRANS_BATTERY_FULL, TRANS_IN_CHARGING_LOCATION: STATE_RECHARGE, TRANS_GO_TO_LOCATION: STATE_GO_TO_CHARGING_LOCATION})
			
			StateMachine.add(STATE_GO_TO_CHARGING_LOCATION, GoToChargingStation(interfaceHelper, protegeHelper), transitions={TRANS_BATTERY_FULL: TRANS_BATTERY_FULL, TRANS_IN_CHARGING_LOCATION: STATE_RECHARGE, TRANS_NOT_IN_CHARGING_LOCATION: STATE_PLAN_TO_CHARGING_LOCATION})
			
			StateMachine.add(STATE_RECHARGE, Recharge(interfaceHelper), transitions={TRANS_BATTERY_FULL: TRANS_BATTERY_FULL, TRANS_BATTERY_LOW: STATE_RECHARGE})
		
		StateMachine.add(STATE_RECHARGING_ROUTINE, sm_recharge, transitions={TRANS_BATTERY_FULL: STATE_START_EXPLORING})
		
	# Create and start the introspection server for visualizing the finite state machine.
	sis = smach_ros.IntrospectionServer('sm_introspection', sm_main, '/SM_ROOT')
	sis.start()

	# Execute the state machine. Note that the `outcome` value of the main Finite State Machine is not used.
	outcome = sm_main.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()
	
# The function that get executed at start time.
if __name__ == '__main__':
    main()  # Initialise and start the ROS node.
