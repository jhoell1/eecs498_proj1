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

class REDRobotApp( JoyApp):

    def __init__(self,right_wheel, left_wheel,*arg,**kw):
        JoyApp.__init__( self,
        confPath="$/cfg/JoyApp.yml",
        ) 
        print "DOES ANYTHING WORK AROUND HERE"
        self.right_wheel = right_wheel
        self.left_wheel = left_wheel
        self.robot = REDRobotDriver(self.right_wheel, #right wheel
                                    self.left_wheel,  #left wheel
                                    self.right_wheel, #laser -- placeholder
                                    self.left_wheel,  #tag -- placeholder
                                    0.6) #torque 

    def onStart(self):
        print "Initialzing the robot"

    def onEvent(self, evt):
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
            # Use superclass to show any other events
            return JoyApp.onEvent(self,evt)


if __name__ == "__main__":

    numservos = 2
    c = L.Cluster()
    c.populate(count =2)

    scr = None

    app = REDRobotApp(c.at.Nx02,c.at.Nx18)
    app.run()
