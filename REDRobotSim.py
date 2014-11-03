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
import os


class REDRobotSim( RobotSimInterface ):
    def __init__(self, *args, **kw):
        RobotSimInterface.__init__(self, *args, **kw)
        '''
        might we want to initialize these to be pointing in the positive y direction?
        '''
        self.heading = 0.0 #absolute heading from arena +x --> information not accesible to simulation 
        self.laserHeading = 0.0 #absolute from arena +x --> information not accesible to simulation
        self.tagAngle = 0.0 #wrt heading
        self.position = asfarray([[0.0],[0.0]])
        self.dNoise = 0.01
        self.aNoise = 0.01
        self.dtheta= .001 #.001
        self.dforward = 0.1
        self.dS=.001
        self.oldTag = 0.0
        self.oldDist = 0.0

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

        angle = -1*angle

        direct = -sign(angle)
        finalHeading = self.heading + angle

        while(finalHeading > pi):
            finalHeading = finalHeading - 2*pi
        while(finalHeading < -pi):
            finalHeading = finalHeading + 2*pi

        while(abs(self.heading-finalHeading) > self.dtheta):
            self.unitRobotRotate(direct)
        self.tagRotate(-1*angle) #rotate tag back after finishing rotation

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

        angle = -1*angle

        direct = sign(angle)
        finalTag = self.tagAngle + angle

        while(finalTag > pi):
            finalTag = finalTag - 2*pi
        while(finalTag < -pi):
            finalTag = finalTag + 2*pi

        while(abs(self.tagAngle-finalTag) > self.dtheta):
            self.unitTagRotate(direct)


    def unitRobotMove(self,direct):

        assert(direct==1 or direct == -1)

        # Compute a vector along the forward direction
        fwd = dot([1,-1,-1,1],self.tagPos)/2
        fwd2 = dot(fwd,[1,1j])
        fwd2 = fwd2 * exp(1j * ((self.heading-self.tagAngle)+randn()*self.aNoise))
        fwd[0] = fwd2.real
        fwd[1] = fwd2.imag
        # Move all tag corners forward by distance, with some noise
        self.tagPos = self.tagPos + fwd[newaxis,:] * self.dforward*direct * (1+randn()*self.dNoise)

        #self.position += dot(asfarray([[cos(self.heading),-sin(self.heading)],[sin(self.heading),cos(self.heading)]]),asfarray([[self.dforward*sign(direct)],0]))
        self.refreshState()


    def unitRobotRotate(self,direct):
        #This will rotate the robot's heading a unit step dtheta, and update all tag points
        #also will update alpha. direct has to be +1 or -1
        assert(direct == 1 or direct == -1)
        self.heading = self.heading+self.dtheta*direct*-1 #+ self.groundInteractionNoise()
        #self.unitLaserRotate(direct*-1)



        self.unitTagRotate(direct)
        self.refreshState()

    def unitTagRotate(self,direct):
        """
        Turn tag by dtheta
        """
        assert(direct==-1 or direct==1)
        self.tagAngle += self.dtheta*direct# + self.servoNoise()


        #print "NOISE:"
        #print self.servoNoise()
        self.refreshState()

    def unitLaserRotate(self, direct):
        #This will rotate the robot's laserheading a unit step dtheta
        #also will update alpha. direct has to be +1 or -1
        assert(direct == 1 or direct == -1)
        self.laserHeading = self.laserHeading+self.dtheta*direct*-1 #3+ self.servoNoise()
        self.refreshState()


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

    #implement this function
    def refreshState( self ):

        #d = [cos(self.laserHeading),sin(self.laserHeading)]
        #lasDir = self.position + d # + add noise
        #self.laserAxis = [self.position,lasDir]
        #print("LASERAXIS:")
        #print(self.laserAxis)


        while(self.tagAngle > pi):
            self.tagAngle = self.tagAngle - 2*pi
        while(self.tagAngle < -pi):
            self.tagAngle = self.tagAngle + 2*pi


        while(self.heading > pi):
            self.heading = self.heading - 2*pi
        while(self.heading < -pi):
            self.heading = self.heading + 2*pi
            

        while(self.laserHeading > pi):
            self.laserHeading = self.laserHeading - 2*pi
        while(self.laserHeading < -pi):
            self.laserHeading = self.laserHeading + 2*pi 

        self.laserAxis = dot([[1,1,0,0],[0,0,1,1]],self.tagPos)/2
        da = dot([1,-1],self.laserAxis)
        self.laserAxis[1] += randn(2) * sqrt(sum(da*da)) * 0.01
        zx = dot(self.laserAxis,[1,1j])*exp(1j * ((self.laserHeading-self.tagAngle)+randn()*self.aNoise))
        self.laserAxis[:,0] = zx.real
        self.laserAxis[:,1] = zx.imag

        z = dot(self.tagPos,[1,1j])
        c = mean(z)
        zr = c + (z-c) * exp(1j * ((self.tagAngle-self.oldTag))) #+randn()*self.aNoise)) # think more about this noise -- maybe gaussian?

        self.tagPos[:,0] = zr.real
        self.tagPos[:,1] = zr.imag


        self.oldTag = self.tagAngle

   
        #output current state:
        #os.system('cls' if os.name == 'nt' else 'clear')
       

        print "Current State: "
        print "heading" + repr(self.heading)
        print "tag Angle: " + repr(self.tagAngle)
        print "Laser Angle" + repr(self.laserHeading)

        #self.laserAxis = dot([[1,1,0,0],[0,0,1,1]],self.tagPos)/2
        #da = dot([1,-1],self.laserAxis)
        #self.laserAxis[1] += randn(2) * sqrt(sum(da*da)) * 0.01
    
