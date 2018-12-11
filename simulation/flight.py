# Basic flight simulation
import numpy as np
from matplotlib import pyplot as plt
from math import *

# Motor definition based on a thrust curve
class SolidMotor:
	# Trust profile is an ordered list of pairs, each having a time in seconds and
	# a thrust in newtons, defining the thrust profile of the engine
	def __init__(self, thrustProfile):
		# Validate thrust profile
		assert(len(thrustProfile) > 1)
		assert(thrustProfile[-1][1] == 0.0)
		for i in range(len(thrustProfile)-1): assert(thrustProfile[i][0] < thrustProfile[i+1][0])
		self.thrustProfile = thrustProfile
		self.reset()

	def reset(self):
		# Init engine state
		self.exhausted = False
		self.burning = False
		self.curThrust = 0.0
		self.burningTime = 0.0
		# Init ongoing thrust
		self.currentThrust = 0.0
		self.nextEventId = 0
		self.nextEventT = 0.0
		self.nextThrust = 0.0

	def step(self, dt):
		if self.exhausted: return
		if self.burning:
			self.burningTime += dt
			# Update burn events
			while self.nextEventT < self.burningTime:
				# Check exhausted motor
				if self.nextEventId == len(self.thrustProfile):
					self.exhausted = True
					self.curThrust = 0.0
					return
				nextEvent = self.thrustProfile[self.nextEventId]
				self.nextEventId += 1
				self.lastEventT = self.nextEventT
				self.nextEventT = nextEvent[0]
				self.lastThrust = self.nextThrust
				self.nextThrust = nextEvent[1]
				self.eventDuration = self.nextEventT - self.lastEventT
			eventT = self.burningTime - self.lastEventT
			deltaThrust = self.nextThrust - self.lastThrust
			self.curThrust = self.lastThrust + (eventT/self.eventDuration) * deltaThrust

	def ignite(self):
		assert(not self.burning)
		self.burning = True;

	def thrust(self):
		return self.curThrust

# Sample motor
def F15Engine():
	thrustCurve = [
		(0.0, 0.0),
		(0.25, 12.5),
		(0.5, 25),
		(0.75, 16.5),
		(1, 15.25),
		(1.25, 15),
		(2.5, 14),
		(3, 13),
		(3.25, 13),
		(3.5,0.0)
	]
	return SolidMotor(thrustCurve)

# Rocket model
class RocketModel:
	def __init__(self, engine, wetMass):
		return

def simulateMotor(model, dt, totalTime):
	model.reset()
	T = [0.0]
	x = [ model.thrust() ]
	model.ignite()
	for t in np.arange(0.0, totalTime, dt):
		model.step(dt)
		T.append(t)
		x.append(model.thrust())
	plt.plot(T,x)
	return