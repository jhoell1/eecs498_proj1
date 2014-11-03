from numpy import*
from REDRobotSim import *
from REDsimTagStreamer import *
from joy import*
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
  up_bdx = 0;
  low_bdx  = 0;
  
  for i in range(0,int(max_x/steps)):
    if(y > data[i]):
      up_bdx = i;
      low_bdx = i-1;
      break

  low_y = data[low_bdx]
  up_y = data[up_bdx]


  if(low_y == y):
    return low_bdx
  if(up_y == y):
    return up_bdx;


  m = (up_y-low_y)/(up_bdx*steps-low_bdx*steps);
  b = up_y-m*up_bdx*steps;


  return (y-b)/m


def get_Filtered_S_Value(snum, splan):
	
	n = 10;
	vals = [0]*n

	for i in range(0,n):
		#read the value at sensor snum
		vals[i] = update_sensor(snum, splan)
		time.sleep(.4)
	sns_dat = mean(vals)
	return lookup_Sensor_Nonlin(sns_dat)


def get_WP(splan):

	ts, w = splan.lastWaypoints
	print ts
	while(len(w) == 0):
		ts,w = splan.lastWaypoints
		if(len(w) != 0):
			return w
	return w

def WP_updated(waypoint_list, splan):
	#returns true if new waypoint list is different length than old wp list
	wp_n = get_WP(splan)
	if(len(wp_n) == len(waypoint_list)):
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
    
        

def update_sensor(num, splan):
	ts,f,b = splan.lastSensor
	while(ts == 0):
   		ts,f,b = splan.lastSensor
   		if(ts):
			if(num == 1):
   				return f
			if(num == 2):
   				return b
			else:
   				return -1;
	if(num == 1):
		return f
	if(num == 2):
		return b
	else:
		return 


def path_find(robot, splan):
    robot.robot_turn(pi/2);
    s1 = get_Filtered_S_Value(1, splan)
    s2 = get_Filtered_S_Value(2, splan);
    th = 10
    while(abs(s1-s2) > th):
		robot.robotMove(step_length, 1);
		s1 = get_Filtered_S_Value(1, splan);
		s2 = get_Filtered_S_Value(2, splan);
    robot.robotTurn(-pi/2);
    dcalib = calib_drift(robot);
    robot.robotTurn(dcalib);
    
    
def calib_drift(robot):
    s1_old = get_Filtered_S_Value(1, splan);
    s2_old = get_Filtered_S_Value(2, splan);
    robot.robotMove(step_length, 1);
    s1_new = get_Filtered_S_Value(1, splan);
    s2_new = get_Filtered_S_Value(2, splan);
    drift_guess1 = asin((s1_old-s1_new)/step_length);
    drift_guess2 = asin((-s2_old+s2_new)/step_length);
    direction = 1;
    if(s2_new > s2_old):
        direction = -1;
    return direciton*(drift_guess1+drift_guess2)/2

# sensor1 should always on the left of moving direction
# sensor2 should always on the right of moving direction

def path_find_init(robot, splan, strmr):
	#start with two sensor reading
	#find direction to go in
	#rotate tag 90 deg to line
	#move robot that way
	s1 = get_Filtered_S_Value(1, splan);
	s2 = get_Filtered_S_Value(2, splan);

	print s1
	print s2

	diff = s1-s2
	direction = 0;
	if(diff > 0):
		direction = 1;
	elif( diff < 0):
		direction = -1;
	else:
		return
	turn_by = pi/45
	print "found initial difference!!!!!"
	print diff
	#rotate tag until difference is maximized
	print("rotating tag until difference is maximized")
	angle = 0;

	robot.tagRotate(direction*turn_by);
	strmr.emitTagMessage()
	angle = angle + turn_by*direction;

	s1 = get_Filtered_S_Value(1, splan);
	s2 = get_Filtered_S_Value(2, splan);


	diffn = s1 - s2;

	print "new difference:\n"
	print diffn*direction
	print "old difference:"
	print diff*direction
	while(diffn*direction > diff*direction):
		print "finally entering while loop"
		robot.tagRotate(direction*turn_by)
		strmr.emitTagMessage()
		s1 = get_Filtered_S_Value(1, splan);
		s2 = get_Filtered_S_Value(2, splan);
		diff = diffn;
		diffn = s1 - s2;
		angle = angle + turn_by*direction
		print "old diff"
		print(diff)
		print "new diff"
		print(diffn)
		time.sleep(1)
	print "angle"
	print angle
	print "rotated tag until maximum difference!!!"
	assert(0)


	#now tag is perpendicular to line
	robot.robotTurn(angle)

	dt = 15;
	print("moving robot forward")
	while(abs(diffn) < dt):
		print ("bump")
		robot.robotMove(step_length, direction)
		s1 = get_Filtered_S_Value(1, splan);
		s2 = get_Filtered_S_Value(2, splan);
		diffn = s1-s2;

	#now robot should be straddling line perpendicularly
	robot.robotTurn(pi/2*direction)		




def controller(strmr, robot, splan):
	#THIS SHOUDLD ALWAYS BE RUNNING. IT WILL KEEP TRACK OF Stuff
	print('entered')
	waypoints = get_WP(splan)
	print('got waypoints')
	w_n = waypoints[0]
	w_c = waypoints[0]

	follow_th = .5

	path_find_init(robot, splan, strmr)

	print("entering loop")
	while(len(waypoints)):
		wp_reached = 0;
		#th = 2;
		while(wp_reached == 0):

			strmr.showSensors()

			strmr.emitTagMessage()
			print("entering inner loop")
			#path_follow(robot, w1, w2)
			robot.robotMove(step_length, -1);
			time.sleep(1)
			print("moved forward")
			wp_reached = WP_updated(waypoints, splan)
        	if(wp_reached == 0):
				s1 = get_Filtered_S_Value(1, splan)
            	s2 = get_Filtered_S_Value(2, splan)

            	if(abs(s1-s2) > follow_th):
                	path_find(robot, splan);
                	time.sleep(1)
                	print("should have reached something")

		if(wp_reached):
			waypoints = get_WP(splan);

			wp_reached = 0;
			w_p = w_c;
			w_c = w_n;
			w_n = waypoints[0]
			
			path_transition(robot, w_p, w_c, w_c, w_n);
			time.sleep(1)
			print("transitioning path")