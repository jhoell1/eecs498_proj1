from numpy import *
# The main program is a JoyApp
from joy import Plan, progress
from waypointShared import *



#move parametersor
turn_by = pi/45
f_steps = 1
max_sens_dif = 70

follow_th = 30

distance_between_sensors = 1


#load initial sensor data. We characterize dhte sensor data.. but it turned out to not be very helpful
def sensorDataInit():
	data = loadtxt('Sensor_NonLin.txt');
	return data





class controller_State():
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
		self.controller_State = controller_State()

		self.autonomous = False 
		self.data = sensorDataInit()
		self.current_WP_list = None
		self.prev_WP_list = None


		self.robot_heading = 0
		self.tag_angle = 0


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
		self.num_reads = 40
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
		self.find_dir = 0
		self.turnt = 0
		self.dir_fixed = 0

		self.maxd = 0
		self.f_turnt = 0

		self.f_dir = 1
		self.find_dir_dir = 1


		self.general_x_direction = 0


		self.only_one_reading = 0
		self.max_one_val = 0
		self.block_find = 0
		#path follow sub_states
		self.ready_for_forward = 1

		#other states
		self.one_wp_reached = 0

	def theo_diff(self):
		diff1 = self.lookup_Sensor_Nonlin(self.sens1) - self.lookup_Sensor_Nonlin(self.sens1 + distance_between_sensors)
		diff2 = self.lookup_Sensor_Nonlin(self.sens1) - self.lookup_Sensor_Nonlin(self.sens1 - distance_between_sensors)

		return(min(abs(diff1), abs(diff2))*.9)


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
		low_bdx = 0
		up_bdx  = 0

		for i in range(0,int(max_x/steps)):
			if(y > self.data[i]):
				up_bdx = i;
				low_bdx = i-1;
				break

		low_y = self.data[low_bdx]
		up_y = self.data[up_bdx]

		if(low_y == y):
			return low_bdx
		if(up_y == y):
			return up_bdx;

		m = (up_y-low_y)/(up_bdx*steps-low_bdx*steps);
		b = up_y-m*up_bdx*steps;

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
		if self.current_WP_list is not None :
			self.state = controller_State.path_follow
			self.prev_WP_list = self.current_WP_list
			self.general_x_direction = sign(self.current_WP_list[1][0] - self.current_WP_list[0][0])
			print "===============intialization complete================="
		else:
			self.state = controller_State.init



	def controller_updateSensorData(self):
		if(self.cur_read == 0):
			self.sens1_prev = self.sens1
			self.sens2_prev = self.sens2

		if(self.cur_read < self.num_reads and not self.valid_s_reads):
			#print "sensor read"
			temp_s1 = self.getSensorVal(1)
			temp_s2 = self.getSensorVal(2)
			self.sens1 = (self.sens1 * self.cur_read + temp_s1)/(self.cur_read + 1.0)
			self.sens2 = (self.sens2 * self.cur_read + temp_s2)/(self.cur_read + 1.0)
			self.cur_read = self.cur_read + 1
			return 0


		elif(self.cur_read == self.num_reads):
			self.valid_s_reads = 1
			self.cur_read = 0
			if(self.sens1 <4 or self.sens2 <4):
				return -1
			else:
				return 1

	def controller_pathFollow(self):
		if(self.ready_for_forward):
			self.robot.robotMove(f_steps, self.f_dir)
			self.valid_s_reads = 0
			self.ready_for_forward = 0
		elif(self.valid_s_reads):

			if(self.sens1 < 5 or self.sens2 < 5):
				self.f_dir = -1*self.f_dir
				self.block_find = 1
			
			elif(abs(self.sens1 - self.sens2) > max_sens_dif):
				#too far off path
				self.state = controller_State.path_find

			if(len(self.current_WP_list) != len(self.prev_WP_list)):
				self.state = controller_State.path_transition
				self.keep_looking = 1

			self.ready_for_forward = 1

	def controller_pathFind(self):
		#go here if we are too far off the path
		self.block_find = 0
		drift_calib_f_step = 2
		if(self.ready_for_forward):
			self.sens1_prev = self.sens1
			self.sens2_prev = self.sens2
			self.robot.robotMove(drift_calib_f_step, 1)
			self.valid_s_reads = 0
			self.ready_for_forward = 0
			self.turnt = 0
			self.dir_fixed = 1

		elif(self.valid_s_reads):


			'''print self.sens1_prev
			print self.sens1
			print self.sens2_prev
			print self.sens2
			print "=-=-=-=-=-==============="
			print self.lookup_Sensor_Nonlin(self.sens1_prev)
			print self.lookup_Sensor_Nonlin(self.sens1)
			print self.lookup_Sensor_Nonlin(self.sens2_prev)
			print self.lookup_Sensor_Nonlin(self.sens2)

			rev_s1_p = self.lookup_Sensor_Nonlin(self.sens1_prev)
			rev_s1 = self.lookup_Sensor_Nonlin(self.sens1)

			rev_s2_p = self.lookup_Sensor_Nonlin(self.sens2_prev)
			rev_s2 = self.lookup_Sensor_Nonlin(self.sens2)

			forward_step_length = .15

			print "trying to do arc sin of"
			print ((rev_s1 - rev_s1_p)/forward_step_length)
			print ((-1*rev_s2 + rev_s2_p)/forward_step_length)

			self.drift_guess1 = arcsin((rev_s1 - rev_s1_p)/forward_step_length)
			self.drift_guess2 = arcsin((-1*rev_s2 + rev_s2_p)/forward_step_length)
			if(self.sens2_prev > self.sens2):
				self.drift_dir = -1
			else:
				self.drift_dir = 1	
			self.robot.robotTurn(self.drift_dir*(self.drift_guess1+self.drift_guess2)/2)
			self.valid_s_reads = 0
			self.state = controller_State.path_follow
			'''
			if(not self.turnt):

				print "-======================== turning 90 degress============="
				self.turnt = 1

				if(self.sens2_prev > self.sens2):
					self.drift_dir = -1
				else:
					self.drift_dir = 1
				self.robot.robotTurn(pi/2*self.drift_dir*-1)

			if(self.turnt):	

				if((min(self.sens1, self.sens2) < min(self.sens1_prev, self.sens2_prev)) and (max(self.sens1, self.sens2) < max(self.sens1_prev, self.sens2_prev)) and max(self.sens1, self.sens2) < 220):
					self.dir_fixed = self.dir_fixed*-1


				if(abs(self.sens1 - self.sens2) < max_sens_dif/2):
					self.robot.robotTurn(self.drift_dir*pi/2)
					self.turnt = 0
					print self.sens1
					print self.sens2
					print "========on the path again.. turn back!!@$%!@$Q!==========="
					self.state = controller_State.path_follow
					self.ready_for_forward = 1

				self.robot.robotMove(1, self.dir_fixed)
				self.valid_s_reads = 0	
				print "====-=-=-=- not on path stil.... keep moving robot forward!!!@$!@$!@---0-"
				print self.general_x_direction

				
	def controller_pathTransition(self):
		#move forward a bit more, then xrotate
		#folow old path until either: both sensors have valid reads
		#or: one sensor reads max
		print "PATH TRANSITIONTAA+SF+++++++++++++++++++++==============================="
		if(self.keep_looking and self.valid_s_reads):		
			self.robot.robotMove(f_steps, 1)
			self.valid_s_reads = 0
			if(len(self.current_WP_list) != len(self.prev_WP_list)):
				self.turn_transition = 1
				self.keep_looking = 0

			if(self.sens1 > 0 and self.sens2 > 0):
				self.turn_transition = 1
				self.keep_looking = 0
				print "BEGIN TRANSITION+____+_+______+++++++++=-=-=-=-="
			elif(self.sens1 > 240 or self.sens2 > 240):
				print "BEGIN TRANSITION+____+_+______+++++++++=-=-=-=-="
				self.turn_transition = 1
				self.keep_looking = 0

		elif(self.turn_transition):
			temp_x_o = self.prev_WP_list[0][0]
			temp_y_o = self.prev_WP_list[0][1]
			temp_x_n = self.prev_WP_list[1][0]
			temp_y_n = self.prev_WP_list[1][1]
			t_slope = (temp_y_n - temp_y_o) / (temp_x_n - temp_x_o)
			old_angle = arctan(t_slope)
			#if(old_angle < 0):
			#	old_angle = 2*pi+old_angle

			temp_x_o = self.current_WP_list[0][0]
			temp_y_o = self.current_WP_list[0][1]
			temp_x_n = self.current_WP_list[1][0]
			temp_y_n = self.current_WP_list[1][1]
			t_slope = (temp_y_n - temp_y_o) / (temp_x_n - temp_x_o)
			new_angle = arctan(t_slope)
			#if(new_angle < 0):
			#	new_angle = 2*pi+new_angle
			self.robot.robotTurn((new_angle - old_angle))
			self.robot.tagRotate((new_angle - old_angle))
			self.general_x_direction = sign(self.current_WP_list[1][0] - self.current_WP_list[0][0]) 
			self.prev_WP_list = self.current_WP_list
			self.valid_s_reads = 0
			self.turn_transition = 0
			self.state = controller_State.path_find
			self.robot.robotMove(8, 1)


	def get_more_reads(self):
		self.robot.robotMove(3,self.find_dir_dir)
		if(self.sens1 < 5 or self.sens2 < 5):
			self.find_dir_dir = -1*self.find_dir_dir
			self.robot.robotMove(6, self.find_dir_dir)

		'''dtag = pi/16
		if(self.valid_s_reads and not self.turn_transition):
			self.robot.tagRotate(dtag)
			self.robot.robotTurn(dtag)
			self.valid_s_reads = 0
			if(self.max_one_val < max(self.sens1, self.sens2) and not self.maxd):
				self.max_one_val = max(self.sens1, self.sens2)
				self.maxd = 0
			else:
				self.maxd = 1
				self.robot.robotTurn(-pi/2)
				self.robot.robotMove(1,1)
				self.f_turnt = 1'''


	def behavior(self):
		while True:

			if(self.initialized):
				#this is going to be something to try tp keep track of direction we are heading in, and direciton we need to be heading in...
				temp = True
				if(self.one_wp_reached):
					temp = True
					#set self.dir_to_wp to difference in x values between previous wp and next wp...

			if(self.autonomous == False):
				yield self.forDuration(1.0/9.0)
				continue

			s_stat = self.controller_updateSensorData()


			if s_stat == 0:
				yield self.forDuration(1/9.0)
				continue
			
			if s_stat == -1 and self.block_find:
				
				self.get_more_reads()
				#if(self.f_turnt):
				#	self.robot.robotTurn(pi/2)
				print "=-=-=-=-=-=-=-=-=-=-looking for two sensor readings=-=-=-=-=-=-=-=-=-=-=-=-=-="

			elif(self.state == controller_State.init):
				print "wait for waypoints"


				self.controller_waitForWaypoints()


			elif(self.state == controller_State.path_follow):
				print "entering path follow"
				self.controller_pathFollow()
			elif(self.state == controller_State.path_find):
				print "entering path find"
				self.controller_pathFind()
			elif(self.state == controller_State.path_transition):
				print "entering path transition"
				self.controller_pathTransition()
				
			yield self.forDuration(1/9.0)
