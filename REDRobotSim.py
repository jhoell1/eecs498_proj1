from gzip import open as opengz
from json import dumps as json_dumps
from numpy import cos,sin,asfarray, dot, c_, newaxis, mean, exp, sum, sqrt, sign
from numpy.linalg import svd
from numpy.random import randn
from scipy import pi
from waypointShared import *
from robotSim import *
from joy import progress

from pdb import set_trace as DEBUG



class REDRobotSim( RobotSimInterface ):
    def __init__(self, *args, **kw):
        RobotSimInterface.__init__(self, *args, **kw)
        self.heading = 0.0 #absolute heading from arena +x --> information not accesible to simulation 
        self.laserHeading = 0.0 #absolute from arena +x --> information not accesible to simulation
        self.tagAngle = 0.0 #wrt heading
        self.position = asfarray([0.0,0.0])
        self.dNoise = 0.01
        self.aNoise = 0.01
        self.dtheta= 0.1 #.001
        self.dforward = 0.1
        self.dS=.001

        self.oldTag = 0.0



    #write functions to control robot

    def robotMove(self,numSteps,direct):
        assert(direct==-1 or direct==1)
        for i in range(0,numSteps):
            self.unitRobotMove(direct)

    def robotTurn(self, angle):
        while (angle > pi):
            angle -= 2*pi;
        while (angle < -pi):
            angle += 2*pi;

        direct = sign(angle)
        finalHeading = self.heading + angle
        while(abs(self.heading-finalHeading) > self.dtheta):
            self.unitRobotRotate(direct)
        self.tagRotate(-angle) #rotate tag back after finishing rotation

    def laserRotate(self,angle):
        while (angle > pi):
            angle -= 2*pi;
        while (angle < -pi):
            angle += 2*pi;

        direct = sign(angle)
        finalLaser = self.laserHeading
        while(abs(self.laserHeading-finalLaser) > self.dtheta):
            self.unitLaserRotate(direct)


    def tagRotate(self, angle):
        while (angle > pi):
            angle -= 2*pi;
        while (angle < -pi):
            angle += 2*pi;

        direct = sign(angle)
        finalTag = self.tagAngle
        while(abs(self.tagAngle-finalTag) > self.dtheta):
            self.unitTagRotate(direct)


    def unitRobotMove(self,direct):
        assert(direct==1 or direct == -1)
        self.position += array([cos(self.heading),sin(self.heading)])*self.dforward*sign(direct)
        self.tagPos += array([1,0])*exp(1j*self.heading)*self.dforward*direct



    def unitRobotRotate(self,direct):
        #This will rotate the robot's heading a unit step dtheta, and update all tag points
        #also will update alpha. direct has to be +1 or -1
        assert(direct == 1 or direct == -1)
        self.heading = self.heading+self.dtheta*direct + self.groundInteractionNoise()
        self.unitLaserRotate(direct*-1)
        self.unitTagRotate(direct)

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
        self.laserHeading = self.laserHeading+self.dtheta*direct + self.servoNoise()


    def servoNoise(self):
        """
        STUB Need to figure out how to model servo noise
        """
        return 0

    #def cameraNoise(self):
    #    """
    #    STUB Need to figure out how to model camera noise
    #   """
    #    return 0

    def groundInteractionNoise(self):
        """
        STUB Need to figure out how to model ground interaction noise i.e slip and stuff like that
        """
        return 0

    #implement this function
    def refreshState( self ):
        """
        Make state ready for use by client.

        ALGORITHM:
        Since the entire robot state is captured by the location of the
        robot tag corners, update the laser axis from the robot tag location 
        """
        print(type(self.laserHeading))
        d = [cos(self.laserHeading),sin(self.laserHeading)]
        lasDir = self.position + d # + add noise
        self.laserAxis = [self.position,lasDir]

        z = dot(self.tagPos,[1,1j])
        c = mean(z)
        zr = c + (z-c) * exp(1j * ((self.tagAngle-self.oldTag)+randn()*self.aNoise)) # think more about this noise -- maybe gaussian?
        self.tagPos[:,0] = zr.real
        self.tagPos[:,1] = zr.imag

        self.oldTag = self.tagAngle
        #self.laserAxis = dot([[1,1,0,0],[0,0,1,1]],self.tagPos)/2
        #da = dot([1,-1],self.laserAxis)
        #self.laserAxis[1] += randn(2) * sqrt(sum(da*da)) * 0.01
    
