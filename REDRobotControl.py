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
	# slope = (w2.y - w1.y) / (w2.x - w1.x)
	# line_anlge = math.atan(slope) # in radians (-pi/2, pi/2)
	# if line_anlge < 0:
	# 	line_anlge = math.pi + line_anlge
	old_s1 = update_sensor(1)
	old_s2 = update_sensor(2)
	robot.robot_forward(step_length)
	new_s1 = update_sensor(1)
	new_s2 = update_sensor(2)
	diff = new_s1 - old_s1
	diff_angle = math.asin(diff/step_length)
	robot.robot_turn(diff_angle)


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

		if(wp_reached):
			wp_reached = 0;
			old_w1, old_w2 = w1, w2
			# update w1, w2, 
			# assume waypoint_list has been changed somewhere else
			w1, w2 = waypont_list[0]
			path_transition(robot, old_w1, old_w2, w1, w2);
