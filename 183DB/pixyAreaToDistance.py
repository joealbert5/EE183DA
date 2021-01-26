import numpy as np
import math
import matplotlib.pyplot as pyplot

x = [0,1,2,3,5,11,22,33,44]
y = [58590, 30562, 19599, 9428, 4199, 986, 201, 73.4, 29]
power = 6
nData = 8

def error(x, xhat):
	err = 0
	for xn, xhatn in zip(x, xhat):
		diff = xn - xhatn
		diff2 = math.pow(diff, 2)
		err += diff2
	return err


npx = np.array(x)
npy = np.array(y)
errors = []

for power in range(3, 11):
	res = np.polyfit(npy, npx, power)
	print('res for power = ' + str(power) + ": ")
	print(res)

	dist = []
	for ydata in y:
		power = 6
		eq = 0
		for coeff in res:
			eq += coeff*math.pow(ydata, power)
			power -= 1
		dist.append(eq)
	e = error(x, dist)
	print(e)
power = 6
res = np.polyfit(npy, npx, power)
print('res for power = ' + str(power) + ": ")
print(res)

dist = []
for ydata in y:
	power = 6
	eq = 0
	for coeff in res:
		eq += coeff*math.pow(ydata, power)
		power -= 1
	dist.append(eq)
e = error(x, dist)
print(e)

pyplot.scatter(y, x, c='y', label='Raw Data')
pyplot.scatter(y, dist)
pyplot.plot(y, dist, label='Regression Data')
pyplot.legend()
pyplot.title('Area on Pixy vs Distance from Pixy')
pyplot.xlabel('Area')
pyplot.ylabel('Distance (inches)')
pyplot.show()
