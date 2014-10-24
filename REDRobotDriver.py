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

class REDRobotDriver():
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
        finalLaser = self.laserHeading + angle
        while(abs(self.laserHeading-finalLaser) > self.dtheta):
            self.unitLaserRotate(direct)

    def tagRotate(self, angle):
        while (angle > pi):
            angle -= 2*pi;
        while (angle < -pi):
            angle += 2*pi;

        direct = sign(angle)
        finalTag = self.tagAngle + angle
        while(abs(self.tagAngle-finalTag) > self.dtheta):
            self.unitTagRotate(direct)


    def unitRobotMove(self,direct):

        assert(direct==1 or direct == -1)





    def unitRobotRotate(self,direct):
        #This will rotate the robot's heading a unit step dtheta, and update all tag points
        #also will update alpha. direct has to be +1 or -1
        assert(direct == 1 or direct == -1)



    def unitTagRotate(self,direct):
        """
        Turn tag by dtheta
        """
        assert(direct==-1 or direct==1)


    def unitLaserRotate(self, direct):
        #This will rotate the robot's laserheading a unit step dtheta
        #also will update alpha. direct has to be +1 or -1
        assert(direct == 1 or direct == -1)



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
        deadzoneError = 0

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
        #wheel skew 

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