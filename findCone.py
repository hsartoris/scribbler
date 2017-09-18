from myro import *
import numpy as np



init('/dev/rfcomm0')
setPicSize('small')

r = 210
b = 80
g = 60

#print rgb2yuv(r-10,b-10,g-10)
#yuv = rgb2yuv(r,b,g)
#print yuv
#print rgb2yuv(r+10, b +10, g+10)

#configureBlob(0,254,90,110, 130,145)#really close lol
#configureBlob(0,255,80,100, 190,210)#far away
configureBlob(0, 254, 80, 110, 170, 200)

p = takePicture()
#p = makePicture("close_left.png")
numPix, avgX,avgY = getBlob()


print numPix
print avgX
print avgY
print getObstacle()
print getWidth(p)
print getHeight(p)

def variance(pic, counterThresh=0, satThresh=0):
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
		minL = 1
		maxL = 0
		for y in range(0, yCount):
			rgb = np.array([getRGB(getPixel(pic, x * xOff, y * yOff))]) / 255.0
			matrix[y][x][0] = (np.amin(rgb) + np.amax(rgb))/2
			lSum += matrix[y][x][0]
			delta = np.amax(rgb) - np.amin(rgb)
			if matrix[y][x][0] < minL: minL = matrix[y][x][0]
			if matrix[y][x][0] > maxL: maxL = matrix[y][x][0]
			if delta == 0:
				matrix[y][x][1] = 0
			else:
				matrix[y][x][1] = delta/(1-abs((2*matrix[y][x][0])-1))
		lSum /= float(yCount)
		counter = 0
		for y in range(0, yCount):
			if (abs(lSum - matrix[y][x][0]) < lt) and matrix[y][x][1] > satThresh:
				#print(abs(lSum - matrix[y][x][0]))
				counter += 1
			if matrix[y][x][0] < lThresh or matrix[y][x][0] > 1 - lThresh:
				counter -= 1

		if counter >= counterThresh:
			#check[x] = counter
			#print("diff: " + str(maxL - minL))
			check[x] = 1
			for i in range(266):
				setRGB(getPixel(pic, x * xOff, i), (255, 0, 0))
	return check


variance(p)

#configureBlob(0,254, 120,130, 125, 135)
show(p)
savePicture(p, "pic.png")

#turnRight(90/90.0,4)
#while(timeRemaining(10)): motors(.7, .4)
stop()


def rgb2yuv(R, G, B):
    Y = int(0.299 * R + 0.587 * G + 0.114 * B)
    U = int(-0.14713 * R - 0.28886 * G + 0.436 * B + 128)
    V = int( 0.615 * R - 0.51499* G - 0.10001 * B + 128)
    return [max(min(v,255),0) for v in (Y, U, V)]
