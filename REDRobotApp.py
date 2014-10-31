import ckbot.logical as L 
from joy import*
from sys import stdout

from REDRobotDriver import REDRobotDriver

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
from sensorPlan import SensorPlan

from REDRobotControlPlan import REDRobotControlPlan

class REDRobotApp( JoyApp):

    def __init__(self,right_wheel, left_wheel,laserServo, tagServo,*arg,**kw):
        JoyApp.__init__( self,
        confPath="$/cfg/JoyApp.yml",
        ) 
        print "DOES ANYTHING WORK AROUND HERE"
        self.robot = REDRobotDriver(right_wheel, #right wheel
                                    left_wheel,  #left wheel
                                    laserServo, #laser -- placeholder
                                    tagServo,  #tag -- placeholder
                                    0.6) #torque

    def onStart(self):
        print "Initialzing the robot"
        self.sensor = SensorPlan(self)
        self.sensor.start()
        self.controller = REDRobotControlPlan(self,self.robot)
        self.controller.start()
        self.timeForUpdate = self.onceEvery(1/20.0)

    def onEvent(self, evt):

        if self.timeForUpdate():
            #get the latest information from sensors and update the controller
            #with those values
            ts,f,b = self.sensor.lastSensor
            ts2, w = self.sensor.lastWaypoint
            if ts:
                self.controller.update_waypoint(w)
            if ts2:
                self.controller.update_sensor_values(f,b)


        if evt.type == KEYDOWN:
            if evt.key == K_UP:
                self.robot.unitRobotMove(1)
                return progress("(say) Move forward")
            elif evt.key == K_DOWN:
                self.robot.unitRobotMove(-1)
                return progress("(say) Move back")
            elif evt.key == K_LEFT:
                self.robot.unitRobotRotate(1)
                return progress("(say) Turn left")
            elif evt.key == K_RIGHT:
                self.robot.unitRobotRotate(-1)
                return progress("(say) Turn right")
            elif evt.key == K_a:
                self.controller.go_autonomous()
                return progress("(say) Going Autonomous!")
            # Use superclass to show any other events
            return JoyApp.onEvent(self,evt)


if __name__ == "__main__":

    #numservos = 2
    #c = L.Cluster()
    #c.populate(count = numservos)

    #scr = None

    app = REDRobotApp(c.at.Nx02,c.at.Nx18,c.at.Nx02,c.at.Nx18)
    app.run()
