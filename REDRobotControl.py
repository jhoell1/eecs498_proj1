import numpy
step_length = 10


def lookup_Sensor_Nonlin(y):
  if(y < 0):
    print "error. x > 0"
    return
  if(y > 255):
    print "sensor output is between 0 and 255"
    return;

  steps = .05;
  max_x = 30;
  min_x = 0;

    
  data = loadtxt('Sensor_NonLin.txt');

  
  for i in range(0,int(max_x/steps)):
    if(y > data[i]):
      up_bdx = i;
      low_bdx = i-1;
      break

  low_y = data[low_bd]
  up_y = data[up_bd]


  if(low_y == y):
    return low_bd
  if(up_y == y):
    return up_bd;


  m = (up_y-low_y)/(up_bd*steps-low_bd*steps);
  b = up_y-m*up_bd*steps;


  return (y-b)/m


def get_Filtered_S_Value(snum):
	
	n = 10;
	vals = [0]*n

	for i in range(0,n):
		#read the value at sensor snum
		vals[i] = update_sensor(snum)
	sns_dat = mean(vals)
	return lookup_Sensor_Nonlin(sns_dat)


def get_WP():
	return current_WP_list


def WP_updated(waypont_list):
	#returns true if new waypoint list is different length than old wp list
	wp_n = get_WP()
	if(wp_n.len == waypoint_list.len):
		return 0;
	return 1;

def get_line_angle(w1, w2):
	slope = (w2.y - w1.y) / (w2.x - w1.x)
	line_anlge = numpy.atan(slope) # in radians (-pi/2, pi/2)
	if line_anlge < 0:
		line_anlge = numpy.pi + line_anlge
	return line_anlge

def path_transition(robot, old_w1, old_w2, w1, w2):
	old_line_angle = get_line_angle(old_w1, old_w2)
	new_line_angle = get_line_angle(w1, w2)
	robot.robot_turn(-(new_line_angle - old_line_angle))
	robot.tag_turn(-(new_line_angle - old_line_angle))

def path_follow(robot, w1, w2):
#dont really need this function anymore
#def path_follow(robot, w1, w2):
	# slope = (w2.y - w1.y) / (w2.x - w1.x)
	# line_anlge = math.atan(slope) # in radians (-pi/2, pi/2)
	# if line_anlge < 0:
	# 	line_anlge = math.pi + line_anlge
	# old_s1 = update_sensor(1)
	# old_s2 = update_sensor(2)
	# robot.robot_forward(step_length)
	# new_s1 = update_sensor(1)
	# new_s2 = update_sensor(2)
	# diff = new_s1 - old_s1
	# diff_angle = math.asin(diff/step_length)
	# robot.robot_turn(diff_angle)
	#### this is a lot like d_calib -- remove i think
	#old_s1 = update_sensor(1)
	#old_s2 = update_sensor(2)
	#robot.robot_forward(step_length)
	#new_s1 = update_sensor(1)
	#new_s2 = update_sensor(2)
	#diff = new_s1 - old_s1
	#diff_angle = math.asin(diff/step_length)
	#robot.robot_turn(diff_angle)
        #######
	
        #s1 = update_sensor(1);
        #s2 = update_sensor(2);
        #dcalib = calib_drift(robot);
        #robot.robot_turn(dcalib);
        #th = .1;
        #while(abs(s1-s2) < th):
         #   robot.robot_forward(step_length);
          #  s1 = update_sensor(1)
           # s2 = update_sensor(2)
    
        

def update_sensor(num):
    if(num == 1):
        return sensor_1
    if(num == 2):
        return sensor_2

def path_find(robot):
    robot.robot_turn(pi/2);
    s1 = get_Filtered_S_Value(1)
    s2 = get_Filtered_S_Value(2);
    th = .01;
    while(abs(s1-s2) > th):
        robot.robot_forward(step_length);
    	s1 = get_Filtered_S_Value(1)
   		s2 = get_Filtered_S_Value(2);
    robot.robot_turn(-pi/2);
    dcalib = calib_drift(robot);
    robot.robot_turn(dcalib);
    
    
def calib_drift(robot):
    s1_old = get_Filtered_S_Value(1);
    s2_old = get_Filtered_S_Value(2);
    robot.robot_forward(step_length);
    s1_new = get_Filtered_S_Value(1);
    s2_new = get_Filtered_S_Value(2);
    drift_guess1 = asin((s1_old-s1_new)/step_length);
    drift_guess2 = asin((-s2_old+s2_new)/step_length);
    direction = 1;
    if(s2_new > s2_old):
        direction = -1;
    return direciton*(drift_guess1+drift_guess2)/2

# sensor1 should always on the left of moving direction
# sensor2 should always on the right of moving direction
def controller(robot):
	#THIS SHOUDLD ALWAYS BE RUNNING. IT WILL KEEP TRACK OF Stuff
	waypoints = get_WP()

	w_n = waypoints[0]
	w_c = waypoints[0]

	follow_th = .5

	while(len(waypoint_list)):
		wp_reached = 0;
		#th = 2;
		while(wp_reached == 0):
			#path_follow(robot, w1, w2)
		    robot.robot_forward(step_length);
		    
		    wp_reached = WP_updated(waypoints)

            
            if(wp_reached == 0):
            	s1 = get_Filtered_S_Value(1)
            	s2 = get_Filtered_S_Value(2)

            	if(abs(s1-s2) > follow_th):
                	path_find(robot);

		if(wp_reached):
			waypoints = get_WP();

			wp_reached = 0;
			w_p = w_c;
			w_c = w_n;
			w_n = waypoints[0]
			
			path_transition(robot, w_p, w_c, w_c, w_n);
