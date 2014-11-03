import ckbot.logical as L 
from joy import*
from sys import stdout

from REDRobotDriver import REDRobotDriver

from gzip import open as opengz
from json import dumps as json_dumps
from numpy import cos,sin,asfarray, dot, c_, newaxis, mean, exp, sum, sqrt, sign, pi
from numpy.linalg import svd
from numpy.random import randn
from scipy import pi
from waypointShared import *
from robotSim import *
from joy import progress

from pdb import set_trace as DEBUG
from sensorPlan import SensorPlan

from REDRobotControlPlan_Improved import REDRobotControlPlan

class REDRobotApp( JoyApp):

    def __init__(self,right_wheel, left_wheel,laserServo, tagServo,*arg,**kw):
        JoyApp.__init__( self,
        confPath="$/cfg/JoyApp.yml",
        ) 
        print "DOES ANYTHING WORK AROUND HERE"
        self.rw = right_wheel
        self.lw = left_wheel
        self.lzr = laserServo
        self.tag = tagServo

    def onStart(self):
        print "Initialzing the robot"
        self.sensor = SensorPlan(self)
        self.sensor.start()
        self.timeForUpdate = self.onceEvery(1/20.0)
        #self.bot = REDRobotDriver(right_wheel, #right wheel
        #                            left_wheel,  #left wheel
        #                            laserServo, #laser -- placeholder
        #                            tagServo,  #tag -- placeholder
        #                            0.6) #torque
        self.bot = REDRobotDriver(app,self.rw,self.lw,self.lzr,self.tag,0.3)
       #self.controller = REDRobotControlPlan(self,self.bot)
        self.bot.start()

    def onEvent(self, evt):

        #if self.timeForUpdate():
            #get the latest information from sensors and update the controller
            #with those values
            #ts,f,b = self.sensor.lastSensor
           #ts2, w = self.sensor.lastWaypoint
            #if ts:
            #    self.controller.update_waypoint(w)
            #if ts2:
            #    self.controller.update_sensor_values(f,b)


        if evt.type == KEYDOWN:
            if evt.key == K_UP:
                self.bot.robotMove(5,1)
                return progress("(say) Move forward")
            elif evt.key == K_DOWN:
                self.bot.robotMove(5,-1)
                return progress("(say) Move back")
            elif evt.key == K_LEFT:
                #self.bot.robotTurn(-pi/2)
                self.bot.unitRobotRotate(1)
                self.bot.unitLaserRotate(1)
                return progress("(say) Turn left")
            elif evt.key == K_RIGHT:
                #self.bot.robotTurn(pi/2)
                self.bot.unitRobotRotate(-1)
                self.bot.unitLaserRotate(-1)
                return progress("(say) Turn right")
            elif evt.key == K_q:
                self.bot.unitTagRotate(1)
                #self.bot.tagRotate(pi/2.0)
                return progress("(say) Turning tag")
            elif evt.key == K_w:
                self.bot.laserRotate(-pi/8.0)
                return progress("(say) Turning laser")
            elif evt.key == K_a:
                self.controller.start()
                self.controller.go_autonomous()
                return progress("(say) Going Autonomous!")
            elif evt.key == K_m:
                self.controller.autonomous = False
            # Use superclass to show any other events
            return JoyApp.onEvent(self,evt)


if __name__ == "__main__":

    numservos = 4
    c = L.Cluster()
    c.populate(count = numservos)

    #scr = None
    robot = dict(count=numservos)
    scr = {}

    #02 is right wheel
    #18 is left wheel
    #15 is laser
    #12 is tag
    app = REDRobotApp(c.at.Nx18,c.at.Nx02,c.at.Nx15,c.at.Nx12, robot = robot,scr = scr)
    app.run()
