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
        self.heading = 0 #absolute heading from arena +x --> information not accesible to simulation 
        self.laserHeading = 0 #absolute from arena +x --> information not accesible to simulation
        self.tagAngle = 0 #wrt heading
        self.position = asfarray([0,0])
        self.dNoise = 0.1
        self.aNoise = 0.1
        self.dtheta=.001
        self.dS=.001
        self.dforward = .01


    #write functions to control robot

    def robot_forward( self, dist):

    def robot_turn (self, angle):

    def rotate_tag (self, angle):

    def rotate_laser (self, angle):

    def rotate_unit(self, direct):

    def forward_unit(self):
        '''
        Is this function supposed to be in here like this?
        alpha isn't part of our state. is this function used to modify tagPos elsewhere
        '''
        x_cnew = x_c + dforward*cos(alpha)
        y_cnew = y_c + dforward*sin(alpha)
        [x1,y1,x2,y2,x3,y3,x4,y4]=update_tag_points(x_cnew, y_cnew, alpha)
        return [x1,y1,x2,y2,x3,y3,x4,y4, x_cnew, y_cnew]

    def unitRobotMove(self,direct):
        '''
        This function will move the robot's tag points a distance dforward in the direction of it's heading  
        '''
        self.tagPos += self.dfowrward  * [cos(self.heading),sin(self.heading)]


    def unitRobotRotate(self,direct):
        #This will rotate the robot's heading a unit step dtheta, and update all tag points
        #also will update alpha. direct has to be +1 or -1
        assert(direct == 1 or direct == -1)
        self.heading = self.heading+self.dtheta*direct + self.groundInteractionNoise()
        self.unitRotateLaser(self,direct*-1)
        self.unitRotateTag(self,direct)

    def unitTagRotate(self,direct):
        """
        Turn tag by dtheta
        """
        assert(direct==-1 or direct==1)
        self.tagAngle += self.dtheta*direct + self.servoNoise()

    def unitLaserRotate(self, direct):
        #This will rotate the robot's laserheading a unit step dtheta
        #also will update alpha. direct has to be +1 or -1
        assert(direct == 1 or direct == -1)
        self.laserheading = self.laserheading+self.dtheta*direct + self.servoNoise()


    def servoNoise(self):
        """
        STUB Need to figure out how to model servo noise
        Things to consider here 
        backlash/play
        Encoder dead zone

        """

        play = .001 #some small value that the servo is allowed move to before motion is detected by the encoder
        electricalNoise = .01*randn() #Electrical gaussian noise in the servo's encoding sensor
        
        #The encoder's this dead zone may introuce additional random error

        return play+electricalNoise+deadzoneError

    #def cameraNoise(self):
    #    """
    #    STUB Need to figure out how to model camera noise
    #   NO! not doing it.
    #   """
    #    return 0
    def diameterError():
        leftDiameter = 5.00
        rightDiameter= 5.00

        dRatio = leftDiameter/rightDiameter

        return dRatio

    def wheelMisalignment():
        """
        This error constant is meant to model the slight shift in alignment that can be brought on by
        wheels that are out of alignment
        """
        wheel skew 

        return 0


    def groundInteractionNoise(self):
        """
        STUB Need to figure out how to model ground interaction noise i.e slip 
        Items to consider:
        Slip on the ground
        This error is parasitic in the sense that it detracts from the actual forward or rotational motion that
        the robot exhibits in the arena.

        """


        slipConstant = .007 #small a small portion of error that is introduced by the robot's wheels slippong on the carpet
        robotTilt = .05 #error caused by the fact that the robot rocks back and forth on it's caster legs

        return -1*(slipConstant+ robotTilt)

    #implement this function
    def refreshState( self ):
        """
        Make state ready for use by client.

        ALGORITHM:
        Since the entire robot state is captured by the location of the
        robot tag corners, update the laser axis from the robot tag location 
        """
        lasDir = self.position + [1,0]*exp(1j*(self.laserHeading))+servoNoise()
        self.laserAxis = [self.position,lasDir]

        z = dot(self.tagPos,[1,1j])
        c = mean(z)
        zr = c + (z-c) * exp(1j * (direct*self.dtheta+randn()*self.aNoise*diameterError()))+wheelMisalignment()
        self.tagPos[:,0] = zr.real
        self.tagPos[:,1] = zr.imag


        #self.laserAxis = dot([[1,1,0,0],[0,0,1,1]],self.tagPos)/2
        #da = dot([1,-1],self.laserAxis)
        #self.laserAxis[1] += randn(2) * sqrt(sum(da*da)) * 0.01
    
