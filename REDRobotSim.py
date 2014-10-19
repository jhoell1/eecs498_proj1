from gzip import open as opengz
from json import dumps as json_dumps
from numpy import asfarray, dot, c_, newaxis, mean, exp, sum, sqrt
from numpy.linalg import svd
from numpy.random import randn
from waypointShared import *
from robotSim import *

from pdb import set_trace as DEBUG

#BLAH

class REDRobotSim( RobotSimInterface ):
  def __init__(self, *args, **kw):
    RobotSimInterface.__init__(self, *args, **kw)
    self.heading = 0 #absolute heading --> information not accesible to simulation
    self.laserHeading = 0 #wrt heading
    self.tagAngle = 0 #wrt heading
    self.dNoise = 0.1
    self.aNoise = 0.1

    #write functions to control robot



  def move( self, dist ):

    """
    Move forward some distance
    """
    # Compute a vector along the forward direction
    fwd = dot([1,-1,-1,1],self.tagPos)/2
    # Move all tag corners forward by distance, with some noise
    self.tagPos = self.tagPos + fwd[newaxis,:] * dist * (1+randn()*self.dNoise)

  def turn( self, angle ):
    """
    Turn by some angle
    """
    z = dot(self.tagPos,[1,1j])
    c = mean(z)
    zr = c + (z-c) * exp(1j * (angle+randn()*self.aNoise))
    self.tagPos[:,0] = zr.real
    self.tagPos[:,1] = zr.imag
    

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
    
