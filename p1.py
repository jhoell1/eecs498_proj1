from joy import*
from sys import stdout

class WheelPlan(Plan):
	def __init__(self, app, direction, *arg, **kw):
		Plan.__init__(self,app, *arg, **kw)
		self.direction = direction


	def turnOff(self):
		self.direction = 0

	def setDir(self, direct):
		self.direction = direct

	def getDir(self):
		return self.direction

	def behavior(self):
		while 1:
			self.mod(self.direction)
			yield self.forDuration(1)


class TurnPlan(Plan):
	def __init__(self, app, p, *arg, **kw):
		Plan.__init__(self,app, *arg, **kw)
		self.pos = p
		self.running = 1


	def turnOff(self):
		self.running = 0

	def turnOn(self):
		self.running = 1

	def behavior(self):
		while 1:
			if self.running:
				self.position(self.pos)
			yield self.forDuration(1)
			if self.running:
				self.position(-1*self.pos)
			yield self.forDuration(1)



class WiggleApp(JoyApp):

	def __init__(self, servospec1, servospec2, s3, direction, *arg, **kw):
		JoyApp.__init__(self, *arg, **kw)
		self.servospec1 = servospec1
		self.servospec2 = servospec2
		self.servospec3 = s3
		self.direction = direction

	def onStart(self):
		self.wheelPlan = WheelPlan(self, self.direction, mod = self.servospec1)
		self.turnPlan = TurnPlan(self, 9000, position = self.servospec2)
		self.turnPlan1 = TurnPlan(self,3000,position = self.servospec3)
		self.wheelPlan.start()
		self.turnPlan1.start()
		self.turnPlan.start()

	def onEvent(self, evt):
		if evt.type == KEYDOWN:
			print describeEvt(evt)
			if evt.key == 113:
				self.wheelPlan.turnOff()
				self.turnPlan.turnOff()
				self.turnPlan1.turnOff()
				return
			if evt.key == 100:
				self.wheelPlan.setDir(int(self.wheelPlan.getDir())*-1)

if __name__ == "__main__":

	numservos = 3
	onespec = "K11/@set_torque"
	twospec = "Nx18/@set_pos"
	threespec = "H12/@set_pos"
	scr = None
	if onespec[:1]==">": scr = {}
	if twospec[:1]==">": scr = {}
	if threespec[:1]==">": scr = {}

	app = WiggleApp(onespec, twospec, threespec, 1, robot = dict(count = numservos), scr=scr)
	app.run()
