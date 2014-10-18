from math import*


def update_tag_points(x_c, y_c, alpha):
	#corner of each tag is 45 degrees away from center
	#this function will update the position of each tag based on
	#the angle alpha the robot is facing wrt. x axis on arena
	# and x,y position of center of the robot
	angles = [pi/4, 3*pi/4, 5*pi/4, 7*pi/4];

	x1 = x_c+r_tag*cos(angles[0]-alpha);
	x2 = x_c+r_tag*cos(angles[1]-alpha);
	x3 = x_c+r_tag*cos(angles[2]-alpha);
	x4 = x_c+r_tag*cos(angles[3]-alpha);
	
	y1 = y_c+r_tag*sin(angles[0]-alpha);
	y2 = y_c+r_tag*sin(angles[1]-alpha);
	y3 = y_c+r_tag*sin(angles[2]-alpha);
	y4 = y_c+r_tag*sin(angles[3]-alpha);

	return [x1,y1,x2,y2,x3,y3,x4,y4];


def rotate_unit(direct, alpha):
	#This will rotate the robot a unit step dtheta, and update all tag points
	#also will update alpha. direct has to be +1 or -1
	assert(direct == 1 or direct == -1); 
	alpha = alpha+dtheta*direct;
	return alpha;

def forward_unit(x_c, y_c, alpha):
	x_cnew = x_c + dforward*cos(alpha);
	y_cnew = y_c + dforward*sin(alpha);
	[x1,y1,x2,y2,x3,y3,x4,y4]=update_tag_points(x_cnew, y_cnew, alpha);
	return [x1,y1,x2,y2,x3,y3,x4,y4, x_cnew, y_cnew];

def rotate(angle, x_c, y_c, alpha, alpha_laser, alpha_tag):
#rotate robot by a certain angle, angle must be in radians
#By convention, first change angle to +- pi
#then set direction of rotation 
#then do a bunch of unit step rotations until the desired alpha is achieved

	if(angle > pi):
		angle = angle - 2*pi;
	if(angle < -pi):
		angle = angle + 2*pi;

	if(angle < 0):
		direct = -1;
	else:
		direct = 1;

	alpha_desired = alpha+angle;

	while(abs(alpha_desired - alpha) > dtheta):

		alpha =rotate_unit(direct, alpha);
		alpha_laser = update_laser_angle(alpha_laser, direct);
		alpha_tag = update_laser_angle(alpha_tag, direct);

	return [alpha, alpha_laser, alpha_tag];

def update_laser_angle(alpha_laser, direct):
	assert(direct == 1 or direct == -1)
	return alpha_laser - dtheta*direct;

def rotate_tag(angle_d,tag_angle, x_c, y_c):
	return update_tag_points(x_c, y_c, tag_angle+angle_d);

def path_follow(tag_angle, robot_angle, x_c, y_c):
	#rotate tag perpendicular to line
	tag_pts = rotate_tag(robot_angle+pi/2-tag_angle, tag_angle,x_c,y_c); 
        s1 = get_s(1);
        s2 = get_s(2);
	#while difference of sensor readings is within threshold, move forward
	while(abs(s1-s2) < forward_th):
		f_move = foward_unit(x_c, y_c, robot_angle);
		tag_pts = f_move[0:7];
		x_c = f_move[8];
		y_c = f_move[9];
                s1 = get_s(1);
                s2 = get_s(2);
                
	path_find();

def get_s(num):
        #get sensor reading
        if(num == 1):
                #get reading from first sensor
                return 5;
        if(num == 2):
                #get reading from second sensor
                return 2.42;

def path_find(x_c, y_c, robot_angle, laser_angle):
	diff_max = 0;
	angle_to_line = 0;
	
	d_turn = .001;
	theta = 0;
	while theta < 2*pi:             
                s1 = get_s(1);
                s2 = get_s(2);         
                if(diff_max < abs(s1-s2)):
                        angle_to_line = theta;
                        diff_max = abs(s1-s2);
                tag_pts = rotate_tag(dtheta, theta, x_c, y_c);
                theta = theta + dtheta;
        rotate(theta - robot_angle, x_c,y_c, robot_angle, laser_angle, theta);
        while(abs(s1-s2) < forward_th):
               f_data = forward_unit(x_c, y_c, robot_angle);
               tag_pts = f_data[0:7];
               x_c = f_data[8];
               y_c = f_data[9];
               s1 = get_s(1);
               s2 = get_s(2);
                
        rotate(theta - robot_angle+pi/2, x_c, y_c, robot_angle, laser_angle, theta);
        


def main():

	#constants
	global ARENA_MAX_X;
	global ARENA_MAX_Y;
	global ARENA_MIN_X;
	global ARENA_MIN_Y;
	global r_tag;
	global r_wheel;
	global r_robot;
	global dtheta;
	global dforward;

	global forward_th

	r_tag = 10;
	forward_th = 1;
	#variables
	x_center = 0;
	y_center = 0;
	alpha = pi/2;
	laser_angle = pi/2;
	tag_angle = 0;

	#unit steps
	dtheta = .001;
	dforward = .001;

	rot_vctr=rotate(pi/4, x_center, y_center, alpha, laser_angle, tag_angle);
	print rot_vctr
	print rot_vctr[len(rot_vctr)-1]*180/pi


main()
