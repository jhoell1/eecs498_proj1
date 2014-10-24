import ckbot.logical as L 
from joy import*
from sys import stdout



if __name__ == "__main__":

	print "Starting!!!"
	c = L.Cluster()
	c.populate(count =2)

	right_wheel = c.at.Nx02;
	left_wheel = c.at.Nx18;
	print "lets goooo"
	#

	#rotate is in # of turns.. keep it less than 1 for now
	rotate = .5
	max_pos = 15000
	direction = 1

	err = 100

	left_pos =  left_wheel.get_pos()
	right_pos = right_wheel.get_pos()

	print "right position start"
	print right_pos
	print "left position start"
	print left_pos

	new_left_pos = left_pos + max_pos*rotate*direction
	new_right_pos = right_pos + max_pos*rotate*direction

	if(new_left_pos > max_pos):
		new_left_pos = -1*(new_left_pos - max_pos)
		print "left pos target is bigger than max_pos so new target is:"
		print new_left_pos
	if(new_left_pos < -1*max_pos):
		new_left_pos = -1*(new_left_pos + max_pos)
		print "left pos target is smaller than max_pos so new target is:"
		print new_left_pos

	if(new_right_pos > max_pos):
		new_right_pos = -1*(new_right_pos - max_pos)
		print "rightpos target is bigger than max_pos so new target is:"
		print new_right_pos
	if(new_right_pos < -1*max_pos):
		new_right_pos = -1*(new_right_pos + max_pos)
		print "right pos target is smaller than max_pos so new target is:"
		print new_right_pos

	right_wheel.set_torque(.4*direction);
	left_wheel.set_torque(.4*direction);
	#while(abs(new_right_pos-right_pos) > err):
	#	right_pos = right_wheel.get_pos();
	#	print right_pos
	#	left_pos = left_wheel.get_pos();
	#	print left_pos
		#time.sleep(.2)


	#time rotation... if we want to do that...
	#right_wheel.set_torque(.4);
	#left_wheel.set_torque(.4);
	#time.sleep(20);
	#right_wheel.set_torque(0);
	#left_wheel.set_torque(0);

	#error test: why we need MX
	a =right_wheel.get_pos()
	print "a"
	print a
	t1 = time.time()
	t2 = time.time()
	timeout = 3
	while(t2<(t1+timeout)):
		t2 = time.time()
		right_wheel.set_torque(-.6)
		left_wheel.set_torque(.6)
		time.sleep(.02)
		b=a
		a = right_wheel.get_pos()
		print "a"
		print a/100.0
		angVel= (a-b/.2)

		print "angular vlocity:"
		print angVel
	t1 = time.time()
	t2 = time.time()
	while(t2<(t1+timeout)):
		t2 = time.time()
		right_wheel.set_torque(.6)
		left_wheel.set_torque(-.6)
		time.sleep(.02)
		b=a
		a = right_wheel.get_pos()
		print "a"
		print a/100.0
		angVel= (a-b/.2)

		print "angular vlocity:"
		print angVel
	right_wheel.set_torque(0)
	left_wheel.set_torque(0)

	print "right position end"
	print right_pos
	print "left position end"
	print left_pos	



	#.at.Nx03.set_pos(0)


	#hile(1==1):
	#c.at.Nx18.set_pos(5000)
	#time.sleep(1.5)
	#c.at.Nx18.set_pos(-5000)
	#time.sleep(1.5)

