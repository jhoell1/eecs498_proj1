import math
step_length = 10


#change math to numpy

def update_WP(waypont_list):
	if(update_WP.list_len != len(waypont_list)):
		update_WP.list_len = len(waypont_list)
		return True
	return False

def get_line_angle(w1, w2):
	slope = (w2.y - w1.y) / (w2.x - w1.x)
	line_anlge = math.atan(slope) # in radians (-pi/2, pi/2)
	if line_anlge < 0:
		line_anlge = math.pi + line_anlge
	return line_anlge

def path_transition(robot, old_w1, old_w2, w1, w2):
	old_line_angle = get_line_angle(old_w1, old_w2)
	new_line_angle = get_line_angle(w1, w2)
	robot.robot_turn(-(new_line_angle - old_line_angle))

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
    s1 = update_sensor(1);
    s2 = update_sensor(2);
    th = .01;
    while(abs(s1-s2) < th):
        robot.robot_forward(step_length);
        s1 = update_sensor(1);
        s2 = update_sensor(2);
    robot.robot_turn(-pi/2);
    dcalib = calib_drift(robot);
    robot.robot_turn(dcalib);
    
    
def calib_drift(robot):
    s1_old = update_sensor(1);
    s2_old = update_sensor(2);
    robot.robot_forward(step_length);
    s1_new = update_sensor(1);
    s2_new = update_sensor(2);
    drift_guess1 = asin((s1_old-s1_new)/step_length);
    drift_guess2 = asin((-s2_old+s2_new)/step_length);
    direction = 1;
    if(s2_new > s2_old):
        direction = -1;
    return direciton*(drift_guess1+drift_guess2)/2

# sensor1 should always on the left of moving direction
# sensor2 should always on the right of moving direction
def controller(robot, waypont_list):
	update_WP.list_len = len(waypont_list)
	(w1, w2) = waypont_list[0]

	while(len(waypoint_list)):
		wp_reached = 0;
		# dth = 2;
		while(wp_reached == 0):
			path_follow(robot, w1, w2)
			wp_reached = update_WP()
                s1 = update_sensor(1);
                s2 = update_sensor(2);
                dcalib = calib_drift(robot);
                robot.robot_turn(dcalib);
                th = .1;
               on_track = (abs(s1-s2) < th):
;
		while(wp_reached == 0):
			#path_follow(robot, w1, w2)
		    robot.robot_forward(step_length);
		    wp_reached = update_WP(waypoint_list)

                    s1 = update_sensor(1)
                    s2 = update_sensor(2)
                    if(abs(s1-s2) > th):
                        path_find(robot);

		if(wp_reached):
			wp_reached = 0;
			old_w1, old_w2 = w1, w2
			# update w1, w2, 
			# assume waypoint_list has been changed somewhere else
			w1, w2 = waypont_list[0]
			path_transition(robot, old_w1, old_w2, w1, w2);
