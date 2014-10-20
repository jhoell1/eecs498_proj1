from gzip import open as opengz
from json import dumps as json_dumps
from numpy import asfarray, dot, c_, newaxis, mean, exp, sum, sqrt
from numpy.linalg import svd
from numpy.random import randn
from scipy import pi
from waypointShared import *
from robotSim import *

from pdb import set_trace as DEBUG



class REDRobotSim( RobotSimInterface ):
  def __init__(self, *args, **kw):
    RobotSimInterface.__init__(self, *args, **kw)
    self.heading = 0 #absolute heading --> information not accesible to simulation 
    self.laserHeading = 0 #wrt heading
    self.tagAngle = 0 #wrt heading
    self.dNoise = 0.1
    self.aNoise = 0.1
    global dtheta=.001;
    global dS=.001;
    


    #write functions to control robot
  
  def unitRotateLaser(self, direct)
	#This will rotate the robot's laserheading a unit step dtheta
	#also will update alpha. direct has to be +1 or -1
	assert(direct == 1 or direct == -1); 
	self.laserheading = self.laserheading+dtheta*direct;

    def robot_forward( self, dist):


    def robot_turn (self, dist):

    def rotate_tag (self, angle):


    def rotate_laser (self, angle):
        








  def moveForward( self, dist ):

    """
    Move forward some distance 
    """
	deltaX = dist*cos(self.heading);
	deltaY = dist*sin(self.heading);
	
 	
	
	self.tagpos = self.tagpos+ ([[deltaX, deltaY], [deltaX, deltaY], [deltaX, deltaY], [deltaX, deltaY]]);

	

 


 def turn( self, angle ):
    """
    Turn by some angle
    """
    z = dot(self.tagPos,[1,1j])
    c = mean(z)
    zr = c + (z-c) * exp(1j * (angle+randn()*self.aNoise))
    self.tagPos[:,0] = zr.real
    self.tagPos[:,1] = zr.imag
    
  def rotate_unit(self, direct):
	#This will rotate the robot's heading a unit step dtheta, and update all tag points
	#also will update alpha. direct has to be +1 or -1
	assert(direct == 1 or direct == -1); 
	self.heading = self.heading+dtheta*direct;
	turn(self, direct* dtheta);


  #implement this function
  def refreshState( self ):
    """
    Make state ready for use by client.
    
    ALGORITHM:
    Since the entire robot state is captured by the location of the
    robot tag corners, update the laser axis from the robot tag location 
    """
    self.laserAxis = dot([[1,1,0,0],[0,0,1,1]],self.tagPos)/2
    da = dot([1,-1],self.laserAxis)
    self.laserAxis[1] += randn(2) * sqrt(sum(da*da)) * 0.01
    
