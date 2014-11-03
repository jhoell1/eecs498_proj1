from gzip import open as opengz
from json import dumps as json_dumps
from numpy import cos,sin,asfarray, dot, c_, newaxis, mean, exp, sum, sqrt, sign, pi, radians
from numpy.linalg import svd
from numpy.random import randn
from scipy import pi
from waypointShared import *
from robotSim import *
from joy import *
import time

from pdb import set_trace as DEBUG

# class wheelPlan ( Plan ):    

#     def __init__(self,app,lw,rw,torque,time,angleStepGuess, *arg,**kw):
#         Plan.__init__(self,app,*arg,**kw)
#         self.lw = lw
#         self.rw = rw
#         self.requestRun = False
#         self.inOperation = False
#         self.t = torque
#         self.time = time
#         self.angleKnown = False
#         self.angle = 0.0 #robot heading
#         self.asg = angleStepGuess # estimate based on data of how much the servo turns with the given torque in the given time

#     def moveForward(self, direction): # left is + angle increase
#         if not self.inOperation:
#             self.requestRun = True
#             self.dir_L = direction
#             self.dir_R = -1*direction

#     def turnSingle(self,direction):
#         if not self.inOperation:
#             self.requestRun = True
#             self.dir_L = direction
#             self.dir_R = direction

#     def calibrateAngleGuess(self):
#         #TODO: AUTOMATE ANGLE GUESS CREATION
#             # Do a bunch of unit turns and see how much the angle changes. throw out any values
#             # that are too large or too small --> easiest way?
#         print "empty"

#     def behavior(self):
#         while True:
#             if self.requestRun is True:
#                 self.inOperation = True
#                 pos_before_L = self.lw.get_pos()/100.0 #get_pos values are in decidegrees
#                 pos_before_R = self.rw.get_pos()/100.0 #get_pos values are in decidegrees

#                 self.lw.set_torque(self.t*self.dir_L)
#                 self.rw.set_torque(self.t*self.dir_R)
#                 yield self.forDuration(self.time)
#                 self.rw.set_torque(0)
#                 self.lw.set_torque(0)
#                 self.requestRun = False
#                 self.inOperation = False
#                 pos_after_L = self.lw.get_pos()/100.0 #get_pos values are in decidegrees
#                 pos_after_R = self.rw.get_pos()/100.0 #get_pos values are in decidegrees

#                 ##handle deadzone stuff
#                 diff_L = pos_after_L - pos_before_L
#                 diff_R = pos_after_R - pos_before_R

#                 print "wheels rotated: " + str((abs(diff_L)+abs(diff_R))/2.0)

#                 if self.angleKnown is True: #angle has been initialized
#                     if abs(diff_L - self.asg) > 0.5: #usually means that the step was into the deadzone, inside the deadzone or out of the deadzone
#                         self.angle += 7.0/13.0*self.asg #increase angle by step guess (which was tuned previously)
#                     else:
#                         self.angle += 7.0/13.0(diff_R+diff_L)/2.0
#                 else:
#                     if abs(diff_L - self.asg) < 0.5: #ideally means that both side of the step was not in the deadzone
#                         self.angle = 7.0/13.0(diff_R+diff_L)/2.0
#                         self.angleKnown = True
#                     else:
#                         print "cannot determine absolute angle yet unfortunately. Will wait for next step"

#                 #correct angle to be between -pi and pi
#                 while (self.angle > pi):
#                     self.angle -= 2*pi
#                 while (self.angle < -pi):
#                     self.angle += 2*pi
#                 print "Current Robot Heading: " + str(self.angle)
#             yield self.forDuration(1/20)


class servoPlan( Plan ):
    def __init__(self,app,servo,torque,time,angleStepGuess, *arg,**kw):
        Plan.__init__(self, app, *arg, **kw )
        self.s = servo
        self.requestRun = False
        self.inOperation = False
        self.t = torque
        self.time = time
        self.angleKnown = False
        self.angle = 0.0
        self.asg = 0.0 #angleStepGuess # estimate based on data of how much the servo turns with the given torque in the given time
        self.calibrate = False

    def turnSingle(self,direction):
        if not self.inOperation:
            self.requestRun = True
            self.dir = direction

    def getResolution(self):
        return self.asg


    def calibrateAngleGuess(self, numSamples):
        #TODO: AUTOMATE ANGLE GUESS CREATION
            # Do a bunch of unit turns and see how much the angle changes. throw out any values
            # that are too large or too small --> easiest way?
        self.calibrate = True
        self.numSamples = numSamples

    def behavior(self):
        while True:

            if self.calibrate is True:
                avg = 0.0
                count = 0
                while count < self.numSamples:

                    pos_before = (self.s.get_pos()/100.0) #get_pos values are in decidegrees
                    pos_before *= pi/180.0
                    self.s.set_torque(self.t*1)
                    yield self.forDuration(self.time)
                    self.s.set_torque(0)
                    pos_after = self.s.get_pos()/100.0 #get_pos values are in decidegrees
                    pos_after *= pi/180.0
                    diff = abs(pos_after - pos_before)
                    if diff >= radians(5.0) and diff <= radians(20.0):
                        avg += abs(diff)
                        count += 1
                    else:
                        print "throwing out deadzone value"
                count = 0
                while count < self.numSamples:

                    pos_before = self.s.get_pos()/100.0 #get_pos values are in decidegrees
                    pos_before *= pi/180.0
                    self.s.set_torque(self.t*-1)
                    yield self.forDuration(self.time)
                    self.s.set_torque(0)
                    pos_after = self.s.get_pos()/100.0 #get_pos values are in decidegrees
                    pos_after *= pi/180.0
                    diff = abs(pos_after - pos_before)
                    if diff >= radians(5.0) and diff <= radians(20.0):
                        avg += abs(diff)
                        count += 1
                        if count == (self.numSamples - 1):
                            self.angle = pos_after
                            self.angleKnown = True
                    else:
                        print "throwing out deadzone value"
                self.asg = avg/20.0
                self.calibrate = False
                print "calibrated value: " + str(self.asg)

            if self.requestRun is True:
                self.inOperation = True

                pos_before = self.s.get_pos()/100.0 #get_pos values are in decidegrees
                pos_before *= pi/180.0
                self.s.set_torque(self.t*self.dir)
                yield self.forDuration(self.time)
                self.s.set_torque(0)
                pos_after = self.s.get_pos()/100.0 #get_pos values are in decidegrees
                pos_after *= pi/180.0
                diff = pos_after - pos_before

                self.requestRun = False
                self.inOperation = False
                ##handle deadzone stuff

                if self.angleKnown is True: #angle has been initialized
                    if abs(abs(diff) - self.asg) > 0.05: #usually means that the step was into the deadzone, inside the deadzone or out of the deadzone
                        self.angle += self.dir*self.asg #increase angle by step guess (which was tuned previously)
                    else:
                        self.angle += self.dir*self.asg #pos_after
                    #correct angle to be between -pi and pi
                else:
                    if abs(abs(diff) - self.asg) < 0.05: #ideally means that both side of the step was not in the deadzone
                        self.angle += self.dir*self.asg#pos_after
                        self.angleKnown = True
                    else:
                        print "cannot determine absolute angle yet unfortunately. Will wait for next step"
                while (self.angle > pi):
                    self.angle -= 2*pi
                while (self.angle < -pi):
                    self.angle += 2*pi
            yield self.forDuration(1/20)



class REDRobotDriver( Plan ):

    def __init__(self,app, right_wheel, left_wheel, laser_servo, tag_servo, torque, *arg, **kw):
        Plan.__init__(self,app,*arg, **kw)
        print "initializing REDRobot driver"

        self.wheel_torque = 0.3
        self.wheel_time = 0.2
        self.wheel_step_guess = 8.0

        self.turret_torque = 0.19
        self.turret_time = 0.125
        self.turret_step_guess = 17.3

        self.laserRotateRequest = False
        self.laserAngleRequest = 0.0
        self.laserResolution = 0.0

        self.tagRotateRequest = False
        self.tagAngleRequest = 0.0
        self.tagResolution = 0.0

        self.robotRotateRequest = False
        self.robotAngleRequest = 0.0
        self.dTheta = 0.0
        self.rotateBack = 0.0

        self.numSteps = 0
        self.robotMoveRequested = False


        #orientation stuffs
        self.turnRequested = False
        self.orientation = 0.0
        self.orientationKnown = False
        self.oldWhR = None
        self.oldWhL = None


        self.tagDir = 1
        self.laserDir = 1
        self.turnDir = 1


        # calibration factor. XXXX TODO XXXX calibrate this!
        self.k =  80.0/90.0 #90.0/80.0


        self.rw = servoPlan(app,right_wheel,self.wheel_torque,self.wheel_time,self.wheel_step_guess)
        self.lw = servoPlan(app,left_wheel,self.wheel_torque,self.wheel_time,self.wheel_step_guess)
        #self.wheels = wheelPlan(app,right_wheel,left_wheel,wheel_torque,wheel_time,wheel_step_guess)
        self.laser_serv = servoPlan(app,laser_servo,self.turret_torque,self.turret_time,self.turret_step_guess)
        self.tag_serv = servoPlan(app,tag_servo,self.turret_torque,self.turret_time,self.turret_step_guess)

        #start individual servo plans
        self.rw.start()
        self.lw.start()
        self.laser_serv.start()
        self.tag_serv.start()


        self.rw.calibrateAngleGuess(10)
        self.lw.calibrateAngleGuess(10)
        self.laser_serv.calibrateAngleGuess(10)
        self.tag_serv.calibrateAngleGuess(10)        

########################################################################################
###                             ROBOT MICRO MOVEMENTS                                ###

    def behavior(self):
        i = 0
        while True:
            if i is 20:
                print "Current Absolute Orientation: " + str(self.orientation)
                i = 0
            #needs to manage the overall robot orientation
            if self.rw.angleKnown and self.lw.angleKnown:
                if self.oldWhR is None and self.oldWhL is None:
                    self.oldWhR = self.rw.angle
                    self.oldWhL = self.lw.angle
                elif not self.robotMoveRequested: # only update angle if we are turning
                    diffR = (self.lw.angle - self.oldWhL)
                    diffL = (self.rw.angle - self.oldWhR)
                    distR = diffR*0.07 # wheel radius
                    distL = diffL*0.07 # wheel radius
                    self.dTheta = (distR+distL)/0.26 # baseline #have no clue why +.....should be negative :/
                    self.dTheta *= self.k #k is calibration factor
                    #if self.dTheta != 0.0:
                       # print "diffR: " + str(diffR) + " diffL: " + str(diffL) + " distR: " + str(distR) + " distL: " + str(distL) + " dTheta: " + str(self.dTheta)
                    self.orientation += self.dTheta
                    self.orientationKnown = True

                    while (self.orientation > pi):
                        self.orientation -= 2*pi
                    while (self.orientation < -pi):
                        self.orientation += 2*pi

                    i += 1
                    self.oldWhR = self.rw.angle
                    self.oldWhL = self.lw.angle
                    self.turnRequested = False

            #handle laser angle rotation
            if self.laserRotateRequest is True:
                self.laserResolution = self.laser_serv.getResolution()
                print "laser angle " + str(self.laser_serv.angle)
                print "laser request: " + str(self.laserAngleRequest)
                print "laser resolution: " + str(self.laserResolution)
                if abs(self.laser_serv.angle - self.laserAngleRequest) > self.laserResolution:
                    self.unitLaserRotate(self.laserDir)
                else:
                    self.laserRotateRequest = False
                    print "Finished laser rotation!"

            #handle tag angle rotation
            if self.tagRotateRequest is True:
                #print "inside tag rotate"
                self.tagResolution = self.tag_serv.getResolution()
                #print "tag angle " + str(self.tag_serv.angle)
                #print "tag request: " + str(self.tagAngleRequest)
                #print "tag resolution: " + str(self.tagResolution)
                if abs(self.tag_serv.angle - self.tagAngleRequest) > self.tagResolution:
                    self.unitTagRotate(self.tagDir)
                else:
                    self.tagRotateRequest = False
                    print "Finished tag rotation!"

            if self.robotRotateRequest is True:
                print self.orientation
                print self.robotAngleRequest
                print self.dTheta
                if abs(self.orientation - self.robotAngleRequest) > 0.25:
                    self.unitRobotRotate(self.turnDir)
                    self.unitLaserRotate(sign(self.robotAngleRequest - self.orientation))
                else:
                    self.robotRotateRequest = False
                    self.tagRotate(self.rotateBack) #-original angle
                    print "Finished robot rotation"


            if self.robotMoveRequested and not self.robotRotateRequest:
                if self.numSteps is not 0:
                    self.unitRobotMove(self.robotMoveDir)
                    self.numSteps -= 1
                    yield self.forDuration(self.wheel_time)
                else:
                    self.robotMoveRequested = False

            yield self.forDuration(1.0/20.0)


    def unitRobotMove(self,direct):
        print "Moving robot forward by unit"
        assert(direct==1 or direct == -1)
        

        self.rw.turnSingle(-1*direct)
        self.lw.turnSingle(direct)

        #self.wheels.moveForward(direct)
        
        # pos_before_r = self.right_wheel.get_pos()
        # pos_before_l = self.left_wheel.get_pos()
        # self.right_wheel.set_torque(self.torque*direct)
        # self.left_wheel.set_torque(-self.torque*direct)
        # time.sleep(self.time_step)
        # self.right_wheel.set_torque(0)
        # self.left_wheel.set_torque(0)
        # pos_after_r = self.right_wheel.get_pos()
        # pos_after_l = self.left_wheel.get_pos()
        # dif = (abs(pos_before_r-pos_after_r) + abs(pos_before_l-pos_after_l))/(2.0*100)
        # print "moved " + str(dif) + " degrees"


    def unitRobotRotate(self,direct):
        print "Rotating robot by unit"
        #This will rotate the robot's heading a unit step dtheta, and update all tag points
        #also will update alpha. direct has to be +1 or -1
        assert(direct == 1 or direct == -1)


        self.turnRequested = True
        self.rw.turnSingle(direct)
        self.lw.turnSingle(direct)
        #self.wheels.turnSingle(direct)



        # self.right_wheel.set_torque(self.torque*direct)
        # self.left_wheel.set_torque(self.torque*direct)
        # time.sleep(self.time_step)
        # self.right_wheel.set_torque(0)
        # self.left_wheel.set_torque(0)


    def unitTagRotate(self,direct):
        """
        Turn tag by dtheta
        """
        assert(direct==-1 or direct==1)
        self.tag_serv.turnSingle(direct)
        # pos_before = self.tag_serv.get_pos()
        # self.tag_serv.set_torque(self.torque*direct)
        # time.sleep(self.time_step)
        # self.tag_serv.set_torque(0)
        # pos_after = self.tag_serv.get_pos()
        # dif = abs(pos_before-pos_after)/(100.0)
        # print "rotated tag " + str(dif) + " degrees"

    def unitLaserRotate(self, direct):
        #This will rotate the robot's laserheading a unit step dtheta
        #also will update alpha. direct has to be +1 or -1
        assert(direct==-1 or direct==1)
        self.laser_serv.turnSingle(direct)
        # pos_before = self.laser_serv.get_pos()

        # self.laser_serv.set_torque(self.torque*direct)
        # time.sleep(self.time_step)
        # self.laser_serv.set_torque(0)

        # pos_after = self.laser_serv.get_pos()
        # dif = abs(pos_before-pos_after)/(100.0)
        # print "rotated laser " + str(dif) + " degrees"

    def robotMove(self,numSteps,direct):
        assert(direct==-1 or direct==1)
        self.numSteps = numSteps
        self.robotMoveRequested = True
        self.robotMoveDir = direct

       # for i in range(0,numSteps):
       #     self.unitRobotMove(direct)

    def robotTurn(self, angle):

        self.robotRotateRequest = True

        self.turnDir = sign(angle)
        self.robotAngleRequest = self.orientation + angle

        while (self.robotAngleRequest > pi):
            self.robotAngleRequest -= 2*pi
        while (self.robotAngleRequest < -pi):
            self.robotAngleRequest += 2*pi

        self.rotateBack = angle

        # while (angle > pi):
        #     angle -= 2*pi;
        # while (angle < -pi):
        #     angle += 2*pi;

        # direct = sign(angle)
        # finalHeading = self.orientation + angle
        # while(abs(self.heading-finalHeading) > self.dtheta):
        #     self.unitRobotRotate(direct)
        #self.tagRotate(-angle) #rotate tag back after finishing rotation

    def laserRotate(self,angle):
        self.laserRotateRequest = True
        self.laserDir = sign(angle)

        self.laserAngleRequest = self.laser_serv.angle + angle
        while (self.laserAngleRequest > pi):
            self.laserAngleRequest -= 2*pi
        while (self.laserAngleRequest < -pi):
            self.laserAngleRequest += 2*pi

        # direct = sign(angle)
        # finalLaser = self.laserHeading + angle
        # while(abs(self.laserHeading-finalLaser) > self.dtheta):
        #     self.unitLaserRotate(direct)

    def tagRotate(self, angle):

        self.tagRotateRequest = True
        self.tagDir = sign(angle)
        self.tagAngleRequest = self.tag_serv.angle + angle
        while (self.tagAngleRequest > pi):
            self.tagAngleRequest -= 2*pi
        while (self.tagAngleRequest < -pi):
            self.tagAngleRequest += 2*pi

        # while (angle > pi):
        #     angle -= 2*pi;
        # while (angle < -pi):
        #     angle += 2*pi;

        # direct = sign(angle)
        # finalTag = self.tagAngle + angle
        # while(abs(self.tagAngle-finalTag) > self.dtheta):
        #     self.unitTagRotate(direct)





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