"""
.. module:: state_helper
  :platform: Unix
  :synopsis: Python module for the helpers used in state_machine.py
  
.. moduleauthor:: Ayan Mazhitov mazhitovayan@gmail.com

Write here
"""



# Import all the necessary libraries
import rospy 
import os
from armor_api.armor_client import ArmorClient
from warden_robot import architecture_name_mapper as anm
import re
import random
import math
from actionlib import SimpleActionClient

from threading import Lock
from std_msgs.msg import Bool
from warden_robot.msg import PlanAction, ControlAction
from warden_robot.srv import SetPose

""" ProtegeHelper class that deals with manipulation and queries in armor_py_api
"""
class ProtegeHelper():
	def __init__(self, ):
		self.client = ArmorClient("example", "ontoRef")
		self.__protege_ind = [('E','D7'), ('E','D6'), ('C1','D6'), ('C1','D1'), ('C1','D5'), ('C1','D2'),('C2','D5'),('C2','D4'), ('C2','D3'), ('C2','D7'),('R3','D3'), ('R1','D1'), ('R2','D2'), 					    ('R4','D4')]
		self.__rooms = ['R' + str(i) for i in range(1,5)]
		self.__corridors = ['C1', 'C2', 'E']
		self.__doors = ['D' + str(i) for i in range(1,8)]
		self.__ontology_name = rospy.get_param(anm.PARAM_ONTOLOGY_NAME, "topological_map.owl")
		self.__path = rospy.get_param(anm.PARAM_ONTOLOGY_PATH, "/root/ros_ws/src/warden_robot/topological_map")
		self.__current_location = ""
		self.LOG_TAG = "Protege-Helper"
	def get_doors_list(self):
		return self.__doors
		
	def get_corridors_list(self):
		return self.__corridors
		
	def get_rooms_list(self):
		return self.__rooms
		
	def get_current_location(self):
		return self.__current_location
		
	def load_topological_map(self):
		path = os.path.join(self.__path, self.__ontology_name)
		if os.path.isfile(path):
			self.client.utils.load_ref_from_file(path, 'http://bnc/exp-rob-lab/2022-23', True, "PELLET", True, False)
			self.client.utils.mount_on_ref()
			self.client.utils.set_log_to_terminal(True)
			return 1
		else:
			rospy.logerr("Cannot find the ontology map file. Check the specified path.")
			return 0
		return 0
	
	def add_individuals(self):
		for pair in self.__protege_ind:
			self.client.manipulation.add_objectprop_to_ind("hasDoor", pair[0], pair[1])
		self.client.call('DISJOINT', 'IND', '', self.__rooms + self.__corridors + self.__doors)
	
	def build_map(self):
		if self.load_topological_map():
			self.add_individuals()
			t = math.floor(rospy.Time.now().to_sec())
			for location in self.__rooms + self.__corridors:
				self.client.manipulation.add_dataprop_to_ind('visitedAt', location, "Long", str(t))
			self.reason()
			return 1
		else:
			rospy.logerr("Unexpected error. Map has not been built.")
			return 0
	
	def decide_next_location(self):
		urgent_locations = self.get_urgent_location()
		reachable_locations = self.canReach()
		if urgent_locations: 
			# Find what URGENT locations are within the range of the robot
			urgent_reachable_locations = [item for item in urgent_locations if item in reachable_locations]
			# Pick something random among these locations
			if urgent_reachable_locations:
				return random.choice(urgent_reachable_locations) 
		# If there are no yet urgent locations
		# Just pick something random among locations that can be reached
		return random.choice(reachable_locations) 
	
	def plan_to_recharge_station(self, charging_station):
		if self.__current_location == charging_station:
			return ""
		if self.__current_location in ['C1', 'C2']:
			return charging_station
		if self.__current_location in ['R1', 'R2']:
			return 'C1'
		if self.__current_location in ['R3', 'R4']:
			return 'C2'
	
	def move_robot(self, location, time):
		if isinstance(location, str):
			if not self.__current_location:
				try:
					self.client.manipulation.add_objectprop_to_ind('isIn', 'Robot1', location)
				except:
					rospy.logerr("Exception thrown try to add object property isIn to the robot")
			else:
				try:
					
					self.client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', location, self.__current_location)
				except:
					rospy.logerr("Exception is thrown try to replace the object property isIn to the robot")
			self.__current_location = location
			timestamp_last = self.client.query.dataprop_b2_ind("visitedAt", location)
			timestamp_ = re.sub("[^0-9]", "", timestamp_last[0])
			try:
				self.client.manipulation.replace_dataprop_b2_ind('visitedAt', location, "Long", str(time), timestamp_)
			except:
				msg = "Exception is thrown trying to replace data property visitedAt to location {0} with value {1}".format(location, time)
				rospy.logerr(msg)
			robot_time_ = self.client.query.dataprop_b2_ind("now", 'Robot1')
			time_ = re.sub("[^0-9]", "", robot_time_[0])
			self.client.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(time), time_)
			self.reason()
			
		else:
			rospy.logerr("The location must be of type string")
	
	def reason(self):
		self.client.utils.apply_buffered_changes()
		self.client.utils.sync_buffered_reasoner()
		
	def canReach(self):
		return self.client.query.objectprop_b2_ind("canReach", 'Robot1')
	
	def get_urgent_location(self):
		return self.client.query.ind_b2_class('URGENT')
		
		
		
class ActionClientHelper:
    # Class constructor, i.e., class initializer. Input parameters are:
    #  - `service_name`: it is the name of the server that will be invoked by this client.
    #  - `action_type`: it is the message type that the server will exchange.
    #  - `done_callback`: it is the name of the function called when the action server completed its computation. If
    #     this parameter is not set (i.e., set to `None`), then only the `self._done_callback` function will be
    #     called when the server completes its computation.
    #  - `feedback_callback`: it is the name of the function called when the action server sends a feedback message. If
    #    this parameter is not set (i.e., set to `None`), then only the `self._feedback_callback` functions will be
    #    called when the server sends a feedback message.
    #  - `mutex`: it is a `Lock` object synchronised with the `done_callback` and `feedback_callback`. If it is not set
    #    (i.e., set to `None`), then a new mutex instance is considered. Set this variable if you want to extends the
    #    synchronization with other classes.
    def __init__(self, service_name, action_type, done_callback=None, feedback_callback=None, mutex=None):
        # Initialise the state of this client, i.e.,  `_is_running`, `_is_done`, and `_results`.
        self.reset_client_states()
        # Set the name of the server to be invoked.
        self._service_name = service_name
        
        # Get or create a new mutex.
        if mutex is None:
            self._mutex = Lock()
        else:
            self._mutex = mutex
        # Instantiate a simple ROS-based action client.
        self._client = SimpleActionClient(service_name, action_type)
        # Set the done and feedback callbacks defined by the class using this client.
        self._external_done_cb = done_callback
        self._external_feedback_cb = feedback_callback
        # Wait for the action server to be alive.
        self._client.wait_for_server()
        self.LOG_TAG = "ActionClient-Helper"
    # Start the action server with a new `goal`. Note this call is not blocking (i.e., asynchronous performed).
    def send_goal(self, goal):
        # A new goal can be given to the action server only if it is not running. This simplification implies that
        # within the ROS architecture no more than one client can use the same server at the same time.
        if not self._is_running:
            # Start the action server.
            self._client.send_goal(goal,
                                   done_cb=self._done_callback,
                                   feedback_cb=self._feedback_callback)
            # Set the client's states.
            self._is_running = True
            self._is_done = False
            self._results = None
        else:
            warn_msg = 'Warning send a new goal, cancel the current request first!'
            rospy.logwarn(anm.tag_log(warn_msg, self.LOG_TAG))

    # Stop the computation of the action server.
    def cancel_goals(self):
        # The computation can be stopped only if the server is actually computing.
        if self._is_running:
            # Stop the computation.
            self._client.cancel_all_goals()
            # Reset the client's state.
            self.reset_client_states()
        else:
            warn_msg = 'Warning cannot cancel a not running service!'
            rospy.logwarn(anm.tag_log(warn_msg, self.LOG_TAG))

    # Reset the client state variables stored in this class.
    def reset_client_states(self):
        self._is_running = False
        self._is_done = False
        self._results = None

    # This function is called when the action server send some `feedback` back to the client.
    def _feedback_callback(self, feedback):
        # Acquire the mutex to synchronise the computation concerning the `feedback` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Eventually, call the method provided by the node that uses this action client to manage a feedback.
            if self._external_feedback_cb is not None:
                self._external_feedback_cb(feedback)

        finally:
            # Realise the mutex to (eventually) unblock ROS-based thread waiting on the same mutex.
            self._mutex.release()

    # This function is called when the action server finish its computation, i.e., it provides a `done` message.
    def _done_callback(self, status, results):
        # Acquire the mutex to synchronise the computation concerning the `done` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Set the client's state
            self._is_running = False
            self._is_done = True
            self._results = results
            # Eventually, call the method provided by the node that uses this action client to manage a result.
            if self._external_done_cb is not None:
                self._external_done_cb(status, results)

        finally:
            self._mutex.release()

    # Get `True` if the action server finished is computation, or `False` otherwise.
    # Note that use this method should do it in a `self._mutex` safe manner.
    def is_done(self):  # they should be mutex safe
        return self._is_done

    # Get `True` if the action server is running, or `False` otherwise.
    # A note that use this method should do it in a `self._mutex` safe manner.
    def is_running(self):
        return self._is_running

    # Get the results of the action server, if any, or `None`.
    def get_results(self):
        if self._is_done:
            return self._results
        else:
            log_err = f'Error: cannot get result for `{self._service_name}`.'
            rospy.logerr(anm.tag_log(log_err, self.LOG_TAG))
            return None



class InterfaceHelper:
    def __init__(self):

        self.mutex = Lock()
        # Retrieve the parameters that we need from the parameter server
        self.charging_time = rospy.get_param(anm.PARAM_BATTERY_TIME_CHARGE, 10.0)
        self.waiting_time = rospy.get_param(anm.PARAM_WAIT_IN_LOCATION, 2.0)
        self._charging_location = rospy.get_param(anm.PARAM_CHARGING_STATION, 'E')
        self.randomness = rospy.get_param(anm.PARAM_RANDOM_ACTIVE,  False)
        # Define the dictionary with attached coordinates to each of the location in the topological map
        self._coord_location_dict = {'E': [10.0, 1.0], 'C1': [8.0, 8.0], 'C2': [12.0, 8.0], 'R1': [4.0, 4.0], 'R2': [4.0, 12.0], 'R3': [2.0, 2.0], 'R4': [2.0, 7.0]}
        # Set the initial state involving the `self._battery_low`, `self._start_interaction` and `self._gesture` variables.
        self.reset_battery()
        rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self._battery_callback)
        # Define the clients for the the plan and control action servers.
        self.planner_client = ActionClientHelper(anm.ACTION_PLANNER, PlanAction, mutex=self.mutex)
        self.controller_client = ActionClientHelper(anm.ACTION_CONTROLLER, ControlAction, mutex=self.mutex)
        self.LOG_TAG = "Interface-Helper"
    # Reset the stimulus, which are stored as states variable fo this class.
    # This function assumes that no states of the Finite State Machine run concurrently.
    def reset_battery(self):
        self.mutex.acquire()
        try:
        	self._battery_low = False
        finally:
        	self.mutex.release()
        publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
        publisher.publish(Bool(self._battery_low))
    	
    def get_coord_dict(self):
    	return self._coord_location_dict
    	
    def get_charging_location(self):
    	return self._charging_location
    # The subscriber to get messages published from the `robot-state` node into the `/state/battery_low/` topic.
    def _battery_callback(self, msg):
        # Acquire the mutex to assure the synchronization with the other subscribers and action clients (this assure data consistency).
        self.mutex.acquire()
        try:
            # Get the battery level and set the relative state variable encoded in this class.
            self._battery_low = msg.data

        finally:
            # Release the mutex to eventually unblock the other subscribers or action servers that are waiting.
            self.mutex.release()


    def is_battery_low(self):
        return self._battery_low

    # Update the current robot pose stored in the `robot-state` node.
    @staticmethod
    def init_robot_pose(point):
        # Eventually, wait for the server to be initialised.
        rospy.wait_for_service(anm.SERVER_SET_POSE)
        try:
            # Call the service and set the current robot position.
            service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
            service(point)  # None that the service `response` is not used.
            log_msg = f'Setting initial robot position ({point.x}, {point.y}) to the `{anm.SERVER_SET_POSE}` node.'
            rospy.loginfo(anm.tag_log(log_msg, "Interface-Helper"))
        except rospy.ServiceException as e:
            err_msg = f'Cannot set current robot position through `{anm.SERVER_SET_POSE}` server. Error: {e}'
            rospy.logerr(anm.tag_log(err_msg, "Interface-Helper"))

