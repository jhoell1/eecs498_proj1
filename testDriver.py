from joy import*
from sys import stdout

from gzip import open as opengz
from json import dumps as json_dumps
from numpy import cos,sin,asfarray, dot, c_, newaxis, mean, exp, sum, sqrt, sign
from numpy.linalg import svd
from numpy.random import randn
from scipy import pi
from waypointShared import *
from robotSim import *
#from joy import progress

from pdb import set_trace as DEBUG



if __name__ == "__main__":

	numservos = 2
	right_wheel = "Nx021"
	left_wheel = "Nx18"

	scr = None
	if onespec[:1]==">": scr = {}
	if twospec[:1]==">": scr = {}


	app = REDRobotDriver(right_wheel, left_wheel, 1, robot = dict(count = numservos), scr=scr)
	app.run()