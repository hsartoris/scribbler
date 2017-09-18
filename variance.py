from myro import *
import numpy as np
import time

init('/dev/rfcomm0')
setPicSize('small')

configureBlob(0, 254, 80, 110, 170, 200)

start = time.time()
pic = takePicture()
print(time.time() - start)
#pic = makePicture("med_mid.png")

#lThresh = .125
lThresh = .15
xCount = 60
yCount = 10
xOff = 420 / xCount
yOff = 266 / yCount
lt = .05
#counterThresh = (yCount/2) + 1
counterThresh = 4
check = [0 for x in range(xCount)]
satThresh = .35

start = time.time()
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
	lSum /= float(yCount)
	counter = 0
	for y in range(0, yCount):
		
		if (abs(lSum - matrix[y][x][0]) < lt) and matrix[y][x][1] > satThresh: 
			counter += 1
		if matrix[y][x][0] < lThresh or matrix[y][x][0] > 1 - lThresh: 
			counter -= 1

	if counter >= counterThresh:
		check[x] = counter
		for i in range(266):
			setRGB(getPixel(pic, x * xOff, i), (255, 0, 0))
			
print(time.time() - start)
print(sum(check))
print(check)
savePicture(pic, "pic.png")
