from myro import *
from random import random
from math import *
import numpy as np
import time
'''
lThresh = .125
xCount = 60
yCount = 8
yPad = 133
xOff = 420 / xCount
yOff = 65 / yCount
lt = .05
'''
def variance(pic, counterThresh=0, satThresh=0):
	lThresh = .125
	xCount = 60
	yCount = 8
	yPad = 133
	xOff = 420 / xCount
	yOff = 65 / yCount
	lt = .05
	if counterThresh == 0:
		counterThresh = (yCount/2) + 1
	check = [0 for x in range(xCount)]

	#satThresh = .35
	matrix = np.zeros(shape = (yCount, xCount, 3), dtype=float)
	for x in range(0, xCount):
		for y in range(0, yCount):
			#col = getRGB(getPixel(pic, x * xOff, y * yOff))
			#matrix[y][x] = hls(col[0], col[1], col[2])
			#matrix[y][x] /= 255.0
			rgb = np.array([getRGB(getPixel(pic, x * xOff, y * yOff + yPad))]) / 255.0
			matrix[y][x][0] = (np.amin(rgb) + np.amax(rgb))/2
			delta = np.amax(rgb) - np.amin(rgb)
			if delta == 0:
				matrix[y][x][1] = 0
			else:
				matrix[y][x][1] = delta/(1-abs((2*matrix[y][x][0])-1))
				#if matrix[y][x][1] == 1.0:
					#matrix[y][x][1] = 0 # hacky
	for x in range(0, xCount):
		counter = 0
		lAvg = np.average(matrix[:, :, 0])
		print("Column " + str(x))
		for y in range(0, yCount):
			r,g,b = getRGB(getPixel(pic, x* xOff, y * yOff + yPad))
			setRGB(getPixel(pic, x * xOff, y * yOff + yPad), (255, 0, 0))
			if (abs(lAvg - matrix[y][x][0]) < lt) or matrix[y][x][1] > satThresh:
				if r > 150 and b < 25:
					counter += 1
				counter += 1
			if b > 150:
				print("too blue")
				counter -= 1
			if matrix[y][x][0] < lThresh:
				''' or matrix[y][x][0] > 1 - lThresh:'''
				print("thresholded")
				counter -= 1
		if counter >= counterThresh:
			check[x] = counter
			#check[x] = 1
			for i in range(266):
				setRGB(getPixel(pic, x * xOff, i), (255, 0, 0))
	return check

def found():
	p = takePicture()
	vari = variance(p, counterThresh=4, satThresh=.35)
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
		vari = variance(p)

if __name__ == "__main__":
	init('/dev/rfcomm0')
	setPicSize('small')
	#turnBy(-10, "deg")
	pic = takePicture()
	p = variance(pic, counterThresh=4, satThresh=.35)
	print(p)
	if sum(p) > 8:
		sound = makeSong("valk.mid")
		playSong(sound)
		found()
	savePicture(pic, "pic.png")
