from numpy import*
# The main program is a JoyApp
from joy import Plan, progress
from enum import Enum
from waypointShared import *


turn_by = pi/45
f_steps = 5
max_sens_dif = 10

follow_th = 30


def sensorDataInit():
	data = loadtxt('Sensor_NonLin.txt');
	return data


class controller_State(Enum):
	init = 5
	path_find_init = 1
	path_follow = 2
	path_find = 3
	path_transition = 4
	error_state = 6
	sensor_calc = 7


class REDRobotControlPlan ( Plan ):

	def __init__( self, app, robotDriver, *arg, **kw ):
		Plan.__init__(self, app, *arg, **kw )
		self.robot = robotDriver

		self.autonomous = False 
		self.data = sensorDataInit()
		self.current_WP_list = None
		self.prev_WP_list = None

		self.sens2_prev = 0
		self.drift_guess1 = 0
		self.drift_guess2 = 0
		self.drift_dir = 1
		self.old_line_angle = 0
		self.new_line_angle = 0
		self.keep_looking = 1
		self.turn_transition = 0

		self.all_WP = 0
		
		#overall controller state
		self.state = 0
		self.prev_state = 0
		self.dir_to_wp = 1
		self.initialized = 0

		#states used to iterate the averaging filter
		self.cur_read = 0
		self.sens1 = 0
		self.sens2 = 0
		self.num_reads = 10
		self.valid_s_reads = 0

		#INIT PATH SUB-STATES
		self.diff = 0
		self.direction = 0
		self.angle = 0
		self.got_init_diff = 0
		self.begin_search = 0
		self.found_max = 0
		self.try_move_to_line = 0
		self.f_dir = 1
		self.sens1_prev = 0
		self.found_dir_to_line = 0
		self.on_line = 0
		self.tried_move = 0
		self.ready_to_move = 1

		#path follow sub_states
		self.ready_for_forward = 1

		#other states
		self.one_wp_reached = 0

	def lookup_Sensor_Nonlin(self,y):
		if(y < 0):
			print "error. x > 0"
			return
		if(y > 255):
			print "sensor output is between 0 and 255"
			return;

		steps = .05;
		max_x = 30;
		min_x = 0;

		for i in range(0,int(max_x/steps)):
			if(y > self.data[i]):
				up_bdx = i;
				low_bdx = i-1;
				break

		low_y = self.data[low_bd]
		up_y = self.data[up_bd]

		if(low_y == y):
			return low_bd
		if(up_y == y):
			return up_bd;

		m = (up_y-low_y)/(up_bd*steps-low_bd*steps);
		b = up_y-m*up_bd*steps;

		return (y-b)/m

	def getSensorVal(self,num):
		if(num == 1):
			return self.sensor_1
		if(num == 2):
			return self.sensor_2
			
	def update_waypoint(self,waypointList):
		self.current_WP_list = waypointList	

	def update_sensor_values(self,f,b):
		self.sensor_1 = f
		self.sensor_2 = b

	def go_autonomous(self):
		self.autonomous = True
		self.state = controller_State.init

	def get_WP(self):
		return self.current_WP_list

    def controller_waitForWaypoints(self):
    	print "=========Attempting Waypoint initialization==========="
        if current_WP_list is None :
            self.state = controller_State.path_find_init
            print "===============intialization complete================="
        else
        	self.state = controller_State.init
        	self.prev_WP_list = self.current_WP_list


    def controller_updateSensorData(self):
    	if(self.cur_read < self.num_reads and not self.valid_s_reads):
				print "sensor read"
				temp_s1 = self.getSensorVal(1)
				temp_s2 = self.getSensorVal(2)
				self.sens1 = (self.sens1 * self.cur_read + temp_s1)/(self.cur_read + 1.0)
				self.sens2 = (self.sens2 * self.cur_read + temp_s2)/(self.cur_read + 1.0)
				self.cur_read = self.cur_read + 1			
				self.valid_s_reads = 0

		elif(self.cur_read == self.num_reads):
			print " found 10 sensor reads"
			self.valid_s_reads = 1
			self.cur_read = 0

	def controller_pathFindInit(self):
		if(self.valid_s_reads and not self.got_init_diff):
			print "finding baseline difference"
			self.diff = self.sens1 - self.sens2
			if(self.diff > 0):
				self.direction = 1
			elif(self.diff < 0):
				self.direction = -1
			else:
				state = controller_State.error_state

			self.robot.tagRotate(self.direction * turn_by)
			self.angle = self.angle + turn_by*self.direction
			self.valid_s_reads = 0
			self.got_init_diff = 1

		elif(self.valid_s_reads and self.got_init_diff):
			print "figure out which direction to turn in"
			temp_d = self.sens1 - self.sens2
			if(abs(temp_d) < self.diff):
				self.direction = self.direction * -1
			self.begin_search = 1
			self.diff = temp_d

		elif(self.begin_search):
			print " looking for dat max"
			self.robot.tagRotate(self.direction * turn_by)
			self.angle = self.angle + turn_by*self.direction
			if(self.valid_s_reads):
				temp_d = self.sens1 - self.sens2

				# we are done here.. we have found the max in the previous step
				if(abs(temp_d) < self.diff):
					self.begin_search = 0
					self.found_max = 1

				self.diff = temp_d
				self.valid_s_reads = 0

		elif(self.found_max):
			print " found dat max"
			#turn the robot by the angle to get it aligned with the tag
			self.robot.robotTurn(self.angle)
			self.try_move_to_line = 1
			self.found_max = 0

		elif(self.try_move_to_line):
			self.sens1_prev = self.sens1
			if(not self.tried_move):
				self.robot.robotMove(f_steps, self.f_dir)
				self.tried_move = 1
				self.valid_s_reads = 0
			if(self.valid_s_reads):
				#if sensor readings are getting smaller, then we are going wrong way
				if(self.sens1 < self.sens1_prev):
					self.f_dir = self.f_dir * -1
				self.found_dir_to_line = 1
				self.try_move_to_line = 0

		elif(self.found_dir_to line):
			#here we move until sensor read the same value---ish
			if(self.ready_to_move)
				self.robot.robotMove(f_steps, self.f_dir)
				self.valid_s_reads = 0
				self.ready_to_move = 0

			elif(self.valid_s_reads):
				if(abs(self.sens1 - self.sens2) < max_sens_dif):
					self.on_line = 1
					self.found_dir_to_line = 0
				self.ready_to_move = 1

		elif(self.on_line):
			#errmm need to figure out which direction to turn perpendicular in...
			#now we need to turn the robot perpendicular to tag.. it should be parallel at this point...
			self.robot.robotTurn(pi/2*self)
			self.state = controller_State.path_follow
			self.valid_s_reads = 0

	def controller_pathFollow(self):
		if(self.ready_for_forward):
			self.robot.robotMove(step_length, 1)
			self.valid_s_reads = 0
			self.ready_for_forward = 0
		elif(self.valid_s_reads):
			if(abs(self.sens1 - self.sens2) > follow_th):
				#too far off path
				self.state = controller_State.path_find

			if(len(self.current_WP_list) != len(self.prev_WP_list)):
				self.state = controller_State.path_transition
				self.keep_looking = 1

			self.ready_for_forward = 1
	def controller_pathFind(self):
		#go here if we are too far off the path
		drift_calib_f_step = 2
		if(self.ready_for_forward):
			self.sens1_prev = self.sens1
			self.sens2_prev = self.sens2
			self.robot.robotMove(drift_calib_f_step, 1)
			self.valid_s_reads = 0
			self.ready_for_forward = 0

		elif(self.valid_s_reads):
			self.drift_guess1 = asin((self.sens1_prev - self.sens1)/drift_calib_f_step)
			self.drift_guess2 = asin((-1*self.sens2_prev + self.sens2)/drift_calib_f_step)
			if(self.sens2_prev > self.sens2):
			self.drift_dir = -1
			else:
				self.drift_dir = 1	
				self.robot.robotTurn(self.drift_dir*(self.drift_guess1+self.drift_guess2)/2)
				self.valid_s_reads = 0
				self.state = controller_State.path_follow
	
	def controller_pathTransition(self):
		#move forward a bit more, then rotate
		#folow old path until either: both sensors have valid reads
		#or: one sensor reads max
		if(self.keep_looking and self.valid_s_reads):		
			self.robot.robotMove(f_steps, 1)
			self.valid_s_reads = 0
			if(self.sens1 < 1 or self.sens2 < 1):
				self.turn_transition = 1
			elif(self.sens1 > 10 or self.sens2 > 10 or self.sens1 == 0 or self.sens2 == 0):
				self.turn_transition = 1

		elif(self.turn_transition):
			temp_x_o = self.prev_WP_list[0][0]
			temp_y_o = self.prev_WP_list[0][1]
			temp_x_n = self.prev_WP_list[1][0]
			temp_y_n = self.prev_WP_list[1][1]
			t_slope = (temp_y_n - temp_y_o) / (temp_x_n - temp_x_o)
			old_angle = numpy.atan(t_slope)
			if(old_angle > 0):
				old_angle = numpy.pi+old_angle

			temp_x_o = self.current_WP_list[0][0]
			temp_y_o = self.current_WP_list[0][1]
			temp_x_n = self.current_WP_list[1][0]
			temp_y_n = self.current_WP_list[1][1]
			t_slope = (temp_y_n - temp_y_o) / (temp_x_n - temp_x_o)
			new_angle = numpy.atan(t_slope)
			if(new_angle > 0):
				new_angle = numpy.pi+new_angle
			self.robot.robotTurn(-(new_angle - old_angle))
			self.robot.tagRotate(-(new_angle - old_angle))
			self.prev_WP_list = self.current_WP_list
			self.valid_s_reads = 0
			self.turn_transition = 0
			self.state = controller_State.path_find

    def behavior(self):
		while True:

			if(self.initialized):
				#this is going to be something to try tp keep track of direction we are heading in, and direciton we need to be heading in...
				if(self.one_wp_reached):
					#set self.dir_to_wp to difference in x values between previous wp and next wp...

			if(self.autonomous == False):
				yield self.forDuration(1.0/20.0)
				continue

			self.controller_updateSensorData()

			if(self.state == controller_State.init):
				self.controller_waitForWaypoints()
			elif(self.state == controller_State.path_find_init):
				self.controller_pathFindInit()
			elif(self.state == controller_State.path_follow):
				self.controller_pathFollow()
			elif(self.state == controller_State.path_find):
				self.controller_pathFind()
			elif(self.state == controller_State.path_transition):
				self.controller_pathTransition()
				
		yield self.forDuration(1.0/20.0)
