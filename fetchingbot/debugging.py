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

#keys.remove('nBlocks: ')
#keys.remove('areaF: ')

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
	plt.ylabel('speed (microseconds)')
	plt.plot(np.arange(0,len(x_errors)), x_errors, c='r', label='left speeds')
	plt.plot(np.arange(0,len(x_errors)), y_errors, c='b', label='right speeds')
	'''plt.plot(np.arange(0,len(x_errors)), np.full(len(x_errors), 10), 'b--', label='y_slack')
	plt.plot(np.arange(0,len(x_errors)), np.full(len(x_errors), -10), 'b--')
	plt.plot(np.arange(0,len(x_errors)), np.full(len(y_errors), 20), 'r--', label='x_slack')
	plt.plot(np.arange(0,len(x_errors)), np.full(len(y_errors), -20), 'r--')
	'''#plt.plot(np.arange(0,len(x_errors)), np.full(len(y_errors), 0), 'g--', label='zero error')
	plt.legend()
	plt.show()

def printKeys(keys):
	f = open('puttyLog.txt', 'r')
	for line in f:
		for key in keys:
			if key in line:
				print(line)

motorSpeeds()
appError()
keys.remove('nBlocks: ')
keys.remove('areaF: ')
keys.remove('scanIterTime: ')
printKeys(keys)