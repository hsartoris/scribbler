# This controller implements behaviors and an fixed priority arbitration
# for them.  Assumes existence of single configured robot.
# Behaviors are implemented as objects.
# Author:  Bobby Mills

from myro import *
from random import random
from math import *
import numpy as np
#init("/dev/rfcomm0")
###############################################################################
lastTurn = 1 # 1:right; 2:left; 0:center

def variance(self, pic, counterThresh=0, satThresh=0):
	lThresh = .125
	xCount = 60
	yCount = 10
	xOff = 420 / xCount
	yOff = 266 / yCount
	lt = .05
	if counterThresh == 0:
		counterThresh = (yCount/2) + 1
	else:
		counterThresh = (yCount * counterThresh) + 1
	check = [0 for x in range(xCount)]

	#satThresh = .35
	matrix = np.zeros(shape = (yCount, xCount), dtype=(float, 2))
	for x in range(0, xCount):
		lSum = 0
		for y in range(0, yCount):
			rgb = np.array([getRGB(getPixel(pic, x * xOff, y * yOff))]) / 255.0
			matrix[y][x][0] = (np.amin(rgb) + np.amax(rgb))/2
			lSum += matrix[y][x][0]
			delta = np.amax(rgb) - np.amin(rgb)
			if delta == 0:
				matrix[y][x][1] = 0
			else:
				matrix[y][x][1] = delta/(1-abs((2*matrix[y][x][0])-1))
				if matrix[y][x][1] == 1.0:
					matrix[y][x][1] = 0
		lSum /= float(yCount)
		counter = 0
		for y in range(0, yCount):
			if (abs(lSum - matrix[y][x][0]) < lt) and matrix[y][x][1] > satThresh:
				counter += 1
			if matrix[y][x][0] < lThresh or matrix[y][x][0] > 1 - lThresh:
				counter -= 1

		if counter >= counterThresh:
			#check[x] = counter
			check[x] = 1
			for i in range(266):
				setRGB(getPixel(pic, x * xOff, i), (255, 0, 0))
	return check


class Behavior(object):
    '''High level class for all behaviors.  Any behavior is a
    subclass of Behavior.'''
    NO_ACTION = 0
    def __init__(self):
        self.state = None # Determines current state of the behavior
        # This governs how it will responds to percepts.
    def check(self):
        '''Return True if this behavior wants to execute
        Return False if it does not.'''
        return False
    def run(self):
        '''Execute whatever this behavior does.'''
        return

	def variance(self, pic):
		lThresh = .125
		xCount = 60
		yCount = 10
		xOff = 420 / xCount
		yOff = 266 / yCount
		lt = .05
		counterThresh = (yCount/2) + 1
		check = [0 for x in range(xCount)]

		start = time.time()
		matrix = np.zeros(shape = (yCount, xCount))
		for x in range(0, xCount):
			lSum = 0
			for y in range(0, yCount):
				rgb = np.array([getRGB(getPixel(pic, x * xOff, y * yOff))]) / 255.0
				matrix[y][x] = (np.amin(rgb) + np.amax(rgb))/2
				lSum += matrix[y][x]
			lSum /= float(yCount)
			counter = 0
			for y in range(0, yCount):
				if (abs(lSum - matrix[y][x]) < lt):
					counter += 1
				if matrix[y][x] < lThresh or matrix[y][x] > 1 - lThresh:
					counter -= 1
			if counter >= counterThresh:
				check[x] = 1
				for i in range(266):
					setRGB(getPixel(pic, x * xOff, i), (255, 0, 0))
		return check


###############################################################################

###############################################################################


class Avoid(Behavior):
    '''Behavior to avoid obstacles.  Simply turns away.'''
    TURN_LEFT = 1
    TURN_RIGHT = 2
    BACKUP_SPEED = 1
    BACKUP_DUR = 0.8
    def __init__(self):
        self.state = Avoid.NO_ACTION
        self.obstacleThresh = 2000
        self.turnspeed = 0.8
        self.turndur = 0.75

    def check(self):
        '''see if there are any obstacles.  If so turn other direction'''
        L, C, R = getObstacle()
        print("Avoid obst",L,C,R)

        if L > self.obstacleThresh or C > self.obstacleThresh:
            self.state = Avoid.TURN_RIGHT
            return True
        elif R > self.obstacleThresh:
            self.state = Avoid.TURN_LEFT
            return True
        else:
            self.state = Avoid.NO_ACTION
        return False


    def run(self):
        '''see if there are any obstacles.  If so turn other direction'''
        print('Avoid')
        backward(self.BACKUP_SPEED,self.BACKUP_DUR) # back up a bit
        if self.state == Avoid.TURN_RIGHT:
            print('turning right')
            turnRight(self.turnspeed,self.turndur)
        elif self.state == Avoid.TURN_LEFT:
            print('turning left')
            turnLeft(self.turnspeed,self.turndur)


###############################################################################



###############################################################################


class Wander(Behavior):
    '''
    Behavior to wander forward.
    Heads in direction that varies a bit each time it executes.
    '''
    WANDER = 1
    OBSTACLE_THRESH = 1000
    MAX_SPEED = 1.0
    MIN_SPEED = 0.1
    DSPEED_MAX = 0.1 # most speed can change on one call

    def __init__(self):
        self.state = Wander.NO_ACTION
        self.lspeed = self.MAX_SPEED # speed of left motor
        self.rspeed = self.MAX_SPEED # speed of right motor

    def check(self):
        '''see if there are any possible obstacles.  If not, then wander.'''
        L, C, R = getObstacle()

        print("Wander obst",L,C,R,(L+C+R)/3.0)
        if (L+C+R)/3.0 < self.OBSTACLE_THRESH:
            self.state = self.WANDER
            return True
        else:
            self.state = self.NO_ACTION
            return False

	def run(self):
		'''Modify current motor commands by a value in range [-0.25,0.25].'''
		print('Wander')
		#p = takePicture()


		'''
		dl = (2 * random() - 1) * self.DSPEED_MAX
		dr = (2 * random() - 1) * self.DSPEED_MAX
		self.lspeed = max(self.MIN_SPEED,min(self.MAX_SPEED,self.lspeed+dl))
		self.rspeed = max(self.MIN_SPEED,min(self.MAX_SPEED,self.rspeed+dr))
		motors(self.lspeed,self.rspeed + .2)
		'''
###############################################################################



###############################################################################

class Search(Behavior):

	def __init__(self):
		self.turnspeed = 0.5
		self.turndur = 0.6
		self.conePos = 0
		self.threshold = 1000
		self.limit = 0

	def check(self):
		return self.limit < 6

	def run(self):
		print("I am searching")
		turnRight(1, .66)
		self.limit += 1



###############################################################################



###############################################################################

class Cone(Behavior):

	PIX_THRESHOLD = 500
	CONE = 2


	def __init__(self):
		#self.state = Cone.NO_ACTION
		self.conePic = 1
		self.conePos = 0
		self.turnspeed = 0.5
		self.turndur = 0.6
		self.lspeed = 0.0
		self.rspeed = 0.0
		self.side = 0
		self.close = False

	def check(self):
		if self.close:
			return False
		configureBlob(0,255,80,100, 190,210)
		pic = takePicture()
		print("I took a picture for Cone")
		numPix, avgX, avgY = getBlob()


		if (np.count_nonzero(variance(self, pic, satThresh=.35)) > 3):
			print("CONE! (variance) ")
			self.close = True
			return True
		elif (numPix >= self.PIX_THRESHOLD):
			print("CONE! (blob)")
			self.conePos = avgX
			return True
		else:
			print("failed through tests on cone")
			print(numPix)
			return False

	def run(self):
		if self.close: return
		#motors(max((float(self.conePos)/427)*.1 + .25, .5), max((1 - float(self.conePos)/427)*.1 + .25, .5))
		if(self.conePos < 215):
			self.side = 0
			motors(.4, .5)
		else:
			self.side = 1
			motors(.5, .4)
		print("going to the cone...")

###############################################################################



###############################################################################

class Push(Behavior):


	PIX_THRESHOLD = 100

	def __init__(self):

		self.turnspeed = 0.5
		self.turndur = 0.6
		self.conePos = 0

	def check(self):
		'''configureBlob(0,254,90,110, 130,145)
		pic = takePicture()
		print("I took another picture")
		numPix, avgX, avgY = getBlob()'''

		a = getObstacle()
		if(((a[0] + a[1] + a[2])/3 > 600) and getStall() == False):# or numPix >= self.PIX_THRESHOLD):
			print("Push!")
			#self.conePos = avgX
			return True
		else:
			#stop()
			return False

	def run(self):
		motors(.4, .4)
		#if self.conePos < 215:
		inFront = True
		a = getObstacle()
		while((a[0] + a[1] + a[2])/3 > 600 and getStall() == False ):
			a = getObstacle()
			'''pic = takePicture()
			print("I took another picture")
			numPix, avgX, avgY = getBlob()
			#while(timeRemaining(1)): motors(.35, .4)
			#while(timeRemaining(1)): motors(.4, .35)
			a = getObstacle()
			if(numPix < self.PIX_THRESHOLD or ((a[0] + a[1] + a[2])/3 < 600 or getStall())):
				break'''


###############################################################################



###############################################################################

class Scan(Behavior):


	PIX_THRESHOLD = 500

	def __init__(self):

		self.turnspeed = 0.7
		self.turndur = .7

	def check(self):
		return True

	def run(self):
		#forward(.4, 2)
		while(timeRemaining(10)):
			if lastTurn == 1 or lastTurn == 0:
				turnRight(self.turnspeed, self.turndur)
			else:
				turnLeft(self.turnspeed, self.turndur)
			p = takePicture()
	 		vari = variance(self, p, satThresh=.4)
			print("Looking for the Cone...")

			if(sum(vari) > 3):
				print("Got it")
				break




###############################################################################



###############################################################################

class Lost(Behavior):

	THRESH = 3

	def __init__(self):

		self.turnspeed = 0.7
		self.turndur = 1

	def check(self):
		a = getObstacle()
		if sum(variance(self, takePicture())) < 3 and ((a[0] + a[1] + a[2])/3 < 600):
			return True
		else:
			return False

	def run(self):
		print("I'm Lost")
		p = takePicture()
		while(sum(variance(self, p)) < 3):
			p = takePicture()
			motors(.3, .3)
		print('not lost')
		stop()

###############################################################################



###############################################################################

class Found(Behavior):

	THRESH = 10

	def __init__(self):

		self.turnspeed = 0.7
		self.turndur = 1

	def check(self):
		p = takePicture()
		if (sum(variance(self, p, satThresh=.35)) > 3):
			return True
		else: return False

	def run(self):
		print("Found!")
		stop()
		p = takePicture()
	 	vari = variance(self, p, satThresh=.35)
		print(sum(vari))
		while(sum(vari) > 3):
			# choose direction
			if (sum(vari[:len(vari)/3]) > sum(vari[len(vari)/3:len(vari)/3 * 2]) and sum(vari[:len(vari)/3]) > sum(vari[len(vari)/3 * 2:])):
				#left
				turnLeft(.3, .5)
				motors(.2, .2)
				lastTurn = 2
			elif (sum(vari[:len(vari)/3]) < sum(vari[len(vari)/3:len(vari)/3 * 2]) and sum(vari[len(vari)/3:len(vari)/3 * 2]) > sum(vari[len(vari)/3 * 2:])):
				#center
				motors(.2, .2)
				lastTurn = 0
			else:
				#right
				turnRight(.3, .5)
				motors(.2, .2)
				lastTurn = 1

			p = takePicture()
			vari = variance(self, p)

###############################################################################



###############################################################################

class Box(Behavior):

	def __init__(self):

		self.turnspeed = 0.7
		self.turndur = 1
		self.complete = False
		self.near = False
		self.threshold = 1000
		self.conePos = 0
		self.counter = 0

	def check(self):
		if self.complete:
			print("Completed box")
		return not self.complete

	def run(self):
		configureBlob(0,254, 120,130, 125, 135)
		p = takePicture()
		vari = variance(self, p, 7.0/8)
		if(np.count_nonzero(variance(self, p, satThresh=.4)) > 3):
			# see the cone
			savePicture(p, "cone.png")
			print("Cone located")
			self.complete = True
			return


		if self.counter >= 6:
			self.complete = True
			return

		if(np.count_nonzero(vari) > 3 and sum(getObstacle())/3 > 300):
			self.near = True

		if(self.near):
			print("Near box")
			#bounce around box
			a = getObstacle()
			while sum(a)/3 > 300:
				turnLeft(.4)
				a =getObstacle()
			turnLeft(.7, .4)
			while sum(a)/3 < 300 and timeRemaining(5):
				motors(.7, .4)
				a = getObstacle()
			if(np.count_nonzero(variance(self, takePicture())) > 3):#there is abox in front of us
				print("there is a box")

				return
			elif(sum(a)/3 < 300): #the neither box nor wall is in front
				while(sum(getObstacle())/3 < 300 and timeRemaining(3)): motors(.5, .5)
				if sum(getObstacle())/3 < 300: self.complete = True
			else:
				turnRight(.7, 1.4)# 90 degree?
				if(np.count_nonzero(variance(self, takePicture())) > 20):
					turnRight(.7, 1)
					while sum(a)/3 < 300 and timeRemaining(10):
						motors(.3, .4)
					self.complete = True
				else:
					motors(.3, .3)
					self.complete = True
					return
		else:
			numPix, avgX, avgY = getBlob()
			if(avgX < 215):
				motors(.4, .5)
			else:
				motors(.5, .4)
			self.counter += 1




		configureBlob(0,255,80,100, 190,210)
		return


###############################################################################



###############################################################################



class Controller(object):

	def __init__(self):

		self.b = Behavior()

		'''Create controller for object-finding robot.'''

		configureBlob(0,255,80,100, 190,210)
		setPicSize('small')

		self.hasSeenCone = False
		#self.win = takePicture()

		self.message = "empty"
		self.avoidBehavior = Avoid()
		self.wanderBehavior = Wander()
		self.coneBehavior = Cone()
		self.pushBehavior = Push()
		self.scanBehavior = Scan()
		self.lostBehavior = Lost()
		self.foundBehavior = Found()
		self.searchBehavior = Search()
		self.boxBehavior = Box()

		self.behaviors = [self.boxBehavior, self.coneBehavior, self.searchBehavior, self.avoidBehavior, self.wanderBehavior]

		self.moreBehaviors = [self.coneBehavior, self.foundBehavior, self.scanBehavior]

        # Order implements priority arbitration.
        #self.behaviors = [self.coneBehavior, self.avoidBehavior, self.wanderBehavior]


    # Decide which behavior, in order of priority
    # has a recommendation for the robot
	def arbitrate(self):
		if (self.hasSeenCone == False):
			for behavior in self.behaviors:
				wantToRun = behavior.check()
				if wantToRun:
					behavior.run()
					if behavior == self.coneBehavior:
						self.hasSeenCone = True
					return # no other behavior runs
		else:
			print("going to morebehaviors")
			for behavior in self.moreBehaviors:
				wantToRun = behavior.check()
				if wantToRun:
					behavior.run()
					return
		#self.foundBehavior.run()

	def run(self):
		setForwardness('fluke-forward')
        # This is the simplest loop.  You may like to stop the bot in other ways.
		for seconds in timer(60): # run for 30 seconds
			self.arbitrate()
		stop()




if __name__ == "__main__":
    init('/dev/rfcomm0')
    ctl = Controller()
    ctl.run()
