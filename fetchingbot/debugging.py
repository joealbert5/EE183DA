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
					x_errors.append(float(line[len(keys[9]):-1]))
				if key == keys[10]:
					y_errors.append(float(line[len(keys[10]):-1]))
				if key == keys[14]:
					z.append(float(line[len(keys[14]):-1]))

	print(x_errors)
	print(y_errors)
	plt.title('Return home base spin 180° debugging data')
	plt.xlabel('Number of iterations')
	plt.ylabel('Heading (degrees)')
	plt.scatter(np.arange(0,len(x_errors)), x_errors, c='r', label='Initial reading')
	plt.plot(np.arange(0,len(y_errors)), np.full(len(y_errors), z[0]), c='g', label='target heading')
	plt.plot(np.arange(0,len(y_errors)), np.full(len(y_errors), z[0] + 7), 'g--', label='target heading slack')
	plt.plot(np.arange(0,len(y_errors)), np.full(len(y_errors), z[0] - 7), 'g--', label='target heading slack')
	plt.plot(np.arange(0,len(y_errors)), y_errors, c='b', label='End reading')
	plt.legend()
	plt.show()

def sweepTesting():
	a = []
	w = []
	x_errors = []
	y_errors = []
	z = []

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
					a.append(float(line[len(keys[9]):-1]))	#sweep left

	print(a)
	print(w)
	print(x_errors)
	print(y_errors)
	print(z)
	plt.title('sweep debugging data')
	plt.xlabel('Number of iterations')
	plt.ylabel('Heading (degrees)')
	plt.plot(np.arange(0,len(x_errors)), np.full(len(x_errors), a[0]), 'y--', label='center of sweep')
	#plt.plot(np.arange(0,len(x_errors)), np.full(len(x_errors), y_errors[0]), 'r--', label='sweep left')
	#plt.plot(np.arange(0,len(x_errors)), np.full(len(x_errors), z[0]), 'g--', label='sweep right')
	plt.plot(np.arange(0,len(x_errors)), x_errors, c='b', label='ahe')
	plt.plot(np.arange(0,len(w)), w, c='y', label='currHeading')
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
keys3 = [keys[14], keys[15], keys[16], keys[17], keys[9], keys[18]]
printKeys(keys3)
sweepTesting()
scan2Testing()