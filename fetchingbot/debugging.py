import math
import csv
import sys
import matplotlib.pyplot as plt
import numpy as np

keys = []

with open('debugging_definitions_csv.csv', 'r') as f:
	reader = csv.reader(f)
	for row in reader:
		print(row)
		keys.append(row[1])
print(keys)

def appError():
	x_errors = []
	y_errors = []

	f = open('puttyLog.txt', 'r')
	for line in f:
		for key in keys:
			if key in line:
				if key == keys[4]:
					x_errors.append(int(line[len(keys[4]):-1]))
				if key == keys[5]:
					y_errors.append(int(line[len(keys[5]):-1]))

	print(np.arange(0,len(x_errors)))
	print(x_errors)
	print(np.full(len(x_errors), 10))
	plt.title('Approach debugging data')
	plt.xlabel('Number of iterations')
	plt.ylabel('Error (coordinate reading - center target)')
	plt.plot(np.arange(0,len(x_errors)), x_errors, c='r', label='x_errors')
	plt.plot(np.arange(0,len(x_errors)), y_errors, c='b', label='y_errors')
	plt.plot(np.arange(0,len(x_errors)), np.full(len(x_errors), 10), 'b--', label='y_slack')
	plt.plot(np.arange(0,len(x_errors)), np.full(len(x_errors), -10), 'b--')
	plt.plot(np.arange(0,len(x_errors)), np.full(len(y_errors), 20), 'r--', label='x_slack')
	plt.plot(np.arange(0,len(x_errors)), np.full(len(y_errors), -20), 'r--')
	#plt.plot(np.arange(0,len(x_errors)), np.full(len(y_errors), 0), 'g--', label='zero error')
	plt.legend()
	plt.show()

def motorSpeeds():
	x_errors = []
	y_errors = []

	f = open('puttyLog.txt', 'r')
	for line in f:
		for key in keys:
			if key in line:
				if key == keys[7]:
					x_errors.append(int(line[len(keys[7]):-1]))
				if key == keys[8]:
					y_errors.append(int(line[len(keys[8]):-1]))

	print(np.arange(0,len(x_errors)))
	print(x_errors)
	print(np.full(len(x_errors), 10))
	plt.title('Motor speeds debugging data')
	plt.xlabel('Number of iterations')
	plt.ylabel('Spin rate (range [0,180])')
	plt.plot(np.arange(0,len(x_errors)), x_errors, c='r', label='left speeds')
	plt.plot(np.arange(0,len(x_errors)), y_errors, c='b', label='right speeds')
	plt.legend()
	plt.show()

def scan2Testing():
	x_errors = []
	y_errors = []
	z = []

	f = open('puttyLog.txt', 'r')
	for line in f:
		for key in keys:
			if key in line:
				if key == keys[9]:
					x_errors.append((len(y_errors), float(line[len(keys[9]):-1])))	#start heading
				if key == keys[10]:
					y_errors.append(float(line[len(keys[10]):-1]))	#currHeading
				if key == keys[14]:
					z.append(float(line[len(keys[14]):-1]))			#target heading

	print(x_errors)
	print(y_errors)
	print(z)
	plt.title('scan2 debugging data')
	plt.xlabel('Number of iterations')
	plt.ylabel('Heading (degrees)')
	plt.scatter([x[0] for x in x_errors], [x[1] for x in x_errors], c='r', label='Initial reading')
	plt.plot(np.arange(0,len(y_errors)), np.full(len(y_errors), z[0]), c='g', label='target heading')
	plt.plot(np.arange(0,len(y_errors)), np.full(len(y_errors), z[0] + 15), 'g--', label='target heading slack')
	plt.plot(np.arange(0,len(y_errors)), np.full(len(y_errors), z[0] - 15), 'g--', label='target heading slack')
	plt.plot(np.arange(0,len(y_errors)), y_errors, c='b', label='currHeading')
	plt.legend()
	plt.show()

def magCenterTest():
	x_errors = []
	y_errors = []
	z = []

	f = open('puttyLog.txt', 'r')
	for line in f:
		for key in keys:
			if key in line:
				if key == keys[19]:
					x_errors.append(float(line[len(keys[19]):-1]))	#sensTx
				if key == keys[20]:
					y_errors.append(float(line[len(keys[20]):-1]))	#sensTy

	print(x_errors)
	print(y_errors)
	print(z)
	z = x_errors
	plt.title('magCenterTest debugging data')
	plt.xlabel('sensTx')
	plt.ylabel('sensTy')
	plt.scatter(x_errors, y_errors, c='r', label='sensor Readings')
	plt.plot(x_errors, y_errors, c='b')
	plt.legend()
	plt.show()

def sweepTesting():
	a = []
	w = []
	x_errors = []
	y_errors = []
	z = []
	end180 = 0,0

	f = open('puttyLog.txt', 'r')
	for line in f:
		for key in keys:
			if key in line:
				if key == keys[10]:
					w.append(float(line[len(keys[10]):-1]))	#currheading
				if key == keys[15]:
					x_errors.append(float(line[len(keys[15]):-1]))	#ahe
				if key == keys[16]:
					y_errors.append(float(line[len(keys[16]):-1]))	#sweep left
				if key == keys[17]:
					z.append(float(line[len(keys[17]):-1]))			#sweep right
				if key == keys[9]:
					a.append(float(line[len(keys[9]):-1]))	#start heading
				if key == keys[22]:
					end180 = (len(w), w[-1])				#(iteration#, heading value) for end 180 spin

	print(w)
	print(x_errors)
	print(y_errors)
	print(z)
	print(a)
	print(end180)
	plt.title('sweep debugging data')
	plt.xlabel('Number of iterations')
	plt.ylabel('Heading (degrees)')
	plt.plot(np.arange(0,len(x_errors)), np.full(len(x_errors), end180[1]), 'y--', label='center of sweep')
	plt.plot(np.arange(0,len(x_errors)), np.full(len(x_errors), y_errors[0]), 'r--', label='sweep left')
	plt.plot(np.arange(0,len(x_errors)), np.full(len(x_errors), z[0]), 'g--', label='sweep right')
	plt.plot(np.arange(0,len(x_errors)), x_errors, c='b', label='ahe')
	plt.plot(np.arange(0,len(w)), w, c='y', label='currHeading')
	plt.scatter(end180[0], end180[1], label='end180marker')
	plt.legend()
	plt.show()

def randTesting(data):
	plt.title('scan2 debugging data')
	plt.xlabel('x_coord')
	plt.ylabel('y_coord')
	spots = [(0,0)]
	spotx = 0
	spoty = 0
	x_data = [x[0] for x in data]
	y_data = [x[1] for x in data]
	for r, h in data:
		spotx += math.cos(h/180*3.14159)*r
		spoty += math.sin(h/180*3.14159)*r
		spots.append((spotx, spoty))
	plt.plot([x[0] for x in spots], [x[1] for x in spots], c='r', label='path')
	plt.scatter(0,0)
	plt.legend()
	plt.show()

def printKeys(keys):
	f = open('puttyLog.txt', 'r')
	for line in f:
		for key in keys:
			if key in line:
				print(line)

#motorSpeeds()
#appError()
'''
keys.remove('nBlocks: ')
keys.remove('areaF: ')
keys.remove('scanIterTime: ')
printKeys(keys)'''
'''keys2 = ["startHeading: ", "currHeading: ", "found ball: ", 
	"return to base", "end turn around", "initHeading: ", "targetHeading: ",
	"difference: "]'''
print(len(keys))
keysReturnBase = [keys[21], keys[22], keys[12], keys[23], keys[24], keys[25]]
printKeys(keysReturnBase)
magCenterTest()
#scan2Testing()
#sweepTesting()

stuff = [(30.00, 90.00),
(29.41, 125.31),
(29.15, 84.09),
(29.15, 157.83),
(29.41, 125.31),
(29.41, 54.69),
(29.07, 49.18),
(29.68, 57.38),
(29.07, -116.57),
(29.12, -164.05),
(29.61, 168.31),
(29.27, -172.15),
(29.12, -164.05),
(28.79, 110.32),
(29.73, 132.27),
(29.53, -151.70),
(29.12, -105.95),
(29.41, -72.18),
(29.12, -105.95),
(29.07, -40.82),
(29.07, -63.43),
(29.00, -133.60),
(29.43, -80.22),
(29.00, -133.60),
(29.02, -88.03),
(29.41, -35.31),
(29.27, -82.15),
(29.27, -82.15),
(29.41, -107.82),
(29.07, -153.43),
(29.73, -70.35),
(29.15, -22.17),
(29.21, 51.95),
(29.53, 118.30),
(29.07, 139.18)]

randTesting(stuff)