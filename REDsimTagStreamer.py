# file simTagStreamer.py simulates a robot in an arena

from sensorPlan import SensorPlan
from REDRobotSim import REDRobotSim
from joy import JoyApp, progress
from joy.decl import *
from waypointShared import WAYPOINT_HOST, APRIL_DATA_PORT
from scipy import pi
from REDRobotControlPlan_Improved import*

class REDRobotSimulatorApp( JoyApp ):
  """Concrete class RobotSimulatorApp <<singleton>>
     A JoyApp which runs the REDRobotSim robot model in simulation, and
     emits regular simulated tagStreamer message to the desired waypoint host.
     
     Used in conjection with waypointServer.py to provide a complete simulation
     environment for Project 1
  """    
  def __init__(self,wphAddr=WAYPOINT_HOST,*arg,**kw):
    JoyApp.__init__( self,
      confPath="$/cfg/JoyApp.yml",
      ) 
    self.srvAddr = (wphAddr, APRIL_DATA_PORT)
    
  def onStart( self ):
    # Set up the sensor receiver plan
    self.sensor = SensorPlan(self)
    self.sensor.start()
    self.robSim = REDRobotSim(fn=None)
    self.timeForStatus = self.onceEvery(1)
    self.timeForLaser = self.onceEvery(1/15.0)
    self.timeForFrame = self.onceEvery(1)


    self.controller = REDRobotControlPlan(self, self.robSim)

    self.c_aut = 0;

 

    progress("Using %s:%d as the waypoint host" % self.srvAddr)



  def showSensors( self ):
    ts,f,b = self.sensor.lastSensor
    if ts:
      progress( "Sensor: %d f %d b %d" % (ts,f,b)  )
    else:
      progress( "Sensor: << no reading >>" )
    ts,w = self.sensor.lastWaypoints
    if ts:
      progress( "Waypoints: %d " % ts + str(w))
    else:
      progress( "Waypoints: << no reading >>" )
  
  def emitTagMessage( self ):
    """Generate and emit and update simulated tagStreamer message"""
    self.robSim.refreshState()
    # Get the simulated tag message
    msg = self.robSim.getTagMsg()
    # Use the sensor socket (dual-use) to send to waypointServer
    self.sensor.sendto(msg, self.srvAddr)
    
  def onEvent( self, evt ):
    # periodically, show the sensor reading we got from the waypointServer
    if self.timeForStatus(): 
      self.showSensors()
    # generate simulated laser readings
    #if self.timeForLaser():
      #progress( self.robSim.logLaserValue(self.now) )
    # update the robot and simulate the tagStreamer
    if self.timeForFrame(): 
      self.emitTagMessage()
      ts,w = self.sensor.lastWaypoints
      if ts:
        self.controller.update_waypoint(w)
      ts,f,b = self.sensor.lastSensor
      if ts:
        self.controller.update_sensor_values(f,b)



    if evt.type == KEYDOWN:

      if evt.key == K_c:


        if(self.c_aut == 0):
          self.controller.start()
          self.controller.go_autonomous()

          self.c_aut = 1


      if evt.key == K_UP:
        self.robSim.robotMove(1,1)
        return progress("(say) Move forward")
      elif evt.key == K_DOWN:
        self.robSim.robotMove(1,-1)
        return progress("(say) Move back")
      elif evt.key == K_LEFT:
        self.robSim.robotTurn(pi/16.0)
        return progress("(say) Turn left")
      elif evt.key == K_RIGHT:
        self.robSim.robotTurn(-pi/16.0)
        return progress("(say) Turn right")
      elif evt.key == K_a:
        self.robSim.laserRotate(pi/16.0)
        return progress("(say) Rotate Laser left")
      elif evt.key == K_s:
        self.robSim.laserRotate(-pi/16.0)
        return progress("(say) Rotate Laser right")
      elif evt.key == K_q:
        self.robSim.tagRotate(pi/16.0)
        return progress("(say) Rotate tag left")
      elif evt.key == K_w:
        self.robSim.tagRotate(-pi/16.0)
        return progress("(say) Rotate tag right")

      elif evt.key == K_x:
        assert(0)

    # Use superclass to show any other events
    return 


if __name__=="__main__":
  print """
  Running the robot simulator

  Listens on local port 0xBAA (2986) for incoming waypointServer
  information, and also transmits simulated tagStreamer messages to
  the waypointServer. 
  """
  import sys
  if len(sys.argv)>1:
      app=REDRobotSimulatorApp(wphAddr=sys.argv[1])
  else:
      app=REDRobotSimulatorApp(wphAddr=WAYPOINT_HOST)
  app.run()

