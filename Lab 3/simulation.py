#simulation
#for high speed, min value ~20, max value is ~1300, then jumps to 8190
#for high accuracy, max value is ~?
import matplotlib.pyplot as pyplot
import random

umax = 10
umin = 0
K = 2

q = .125
r = 32
p = 102


def parseData(data):
	xdata = []
	ydata = []
	for x in range(0,len(data)):
		if x % 2 == 0:
			xdata.append(data[x])
		else:
			ydata.append(data[x])
	alldata = {"x":xdata, "y":ydata}
	return alldata

def rawToUnit(data):
	#float m = .0078125
	#float b = -.15625
	newData = []
	for raw in data:
		if raw > 2000:
			return 0
		m = (umax - umin)/1280
		b = -20*m + umin
		y = m*raw + b
		newData.append(y)
	return newData

def getFilteredValue(measurement, x, q, r, p):
      #Updates and gets the current measurement value */
      #prediction update
      #predicted error covariance = previous + process noise
      p = p + q;
    

      #measurement update
	  #gain = ratio between how large sensor noise is compared to previous estimated error
      k = p / (p + r);

      #current filtered value = previous filtered value + gain*(unfiltered - filtered value)
      x = x + k * (measurement - x);

      #current error = (1 - gain)*previous error
      p = (1 - k) * p;
      
      return x;
    

def main():
	#unitData = rawToUnit(data)
	alldata = parseData(data)
	xdata = alldata["x"]
	ydata = alldata["y"]
	noise = 20
	for a in range(0, len(xdata)):
		xdata[a] = xdata[a] + random.randint(-1*noise, noise)
		ydata[a] = ydata[a] + random.randint(-1*noise, noise)
	color = []
	xfilter = []
	xprev = xdata[0]
	yfilter = []
	yprev = ydata[0]
	xfilter2 = []
	xprev2 = xdata[0]
	yfilter2 = []
	yprev2 = ydata[0]
	for i in range(0, len(xdata)): color.append(i)
	for j in range(0, len(xdata)):
		xnext = ((K - 1)/K)*xprev + (1/K)*xdata[j]
		ynext = ((K - 1)/K)*yprev + (1/K)*ydata[j]
		xfilter.append(xnext)
		yfilter.append(ynext)
		xprev = xnext
		yprev = ynext
	for k in range(0, len(xdata)):
		xnext = getFilteredValue(xdata[k], xprev2, q, r, p)
		ynext = getFilteredValue(ydata[k], yprev2, q, r, p)
		xfilter2.append(xnext)
		yfilter2.append(ynext)
		xprev2 = xnext
		yprev2 = ynext
	#pyplot.scatter(xdata, ydata, c=color, s=16)
	pyplot.scatter(xdata, ydata, s=8)
	#pyplot.set_cmap('hot')
	#pyplot.plot(xdata, ydata, 'b-')
	pyplot.plot(xfilter, yfilter, 'r--')
	pyplot.plot(xfilter2, yfilter2, 'g-.')
	pyplot.show()




#in general, assuming sensors are perpendicular to walls (rotation = 0 degrees)
#if coord <= 1300
	#we are proportionally that far away from a wall
	#functionConvertRawDataToUnits
	#small error
	#high certainty
#if coord == 8190
	#in some unknown middle zone
	#need to rely on previous measurements, wheel rotations to guess locaiton



#measure x,y
#guess initial position
"""
data = [1100,1100, 
1100,1100, 
1100,1100, 
1100,1100, 
1100,1100, 
1100,1100, 
1100,1100, 
1100,1100, 
869,1100, 
778,1100, 
630,1100, 
526,1100, 
527,1100, 
496,1100, 
440,1100, 
425,1100, 
423,1100, 
391,1100, 
353,1100, 
363,1100, 
335,1100, 
311,1100, 
307,1100, 
298,1100, 
284,1100, 
271,1100, 
265,1100, 
243,1100, 
238,1100, 
225,1100, 
215,1100, 
203,1100, 
188,1100, 
180,1100, 
174,1100, 
164,1100, 
152,1100, 
148,1100, 
137,1100, 
123,1100, 
118,1100, 
118,1100, 
122,1100, 
132,1100, 
140,1100, 
154,1100, 
154,1100, 
168,1100, 
181,1100, 
189,1100, 
203,1100, 
212,1100, 
232,1100, 
247,1100, 
271,1100, 
283,1100, 
303,1100, 
328,1100, 
340,1100, 
381,1100, 
417,1100, 
422,1100, 
432,1100, 
526,1100, 
578,1100, 
632,1100, 
737,1100, 
901,1100, 
997,1100,  
1100,1100, 
1100,1100, 
1100,1100, 
1100,1100, 
1100,1100, 
1100,198, 
1100,159, 
1100,154, 
1100,153, 
1100,140, 
1100,136, 
1100,133, 
1100,136, 
1100,137, 
1100,147, 
1100,140, 
1100,141, 
1100,137, 
1100,142, 
1100,133, 
1100,142, 
1100,142, 
1100,153, 
1100,153, 
1100,160, 
1100,174, 
1100,179, 
1100,184, 
1100,205, 
1100,220, 
1100,240, 
1100,255, 
1100,266, 
1100,291, 
1100,312, 
1100,309, 
1100,334, 
1100,341, 
1100,353, 
1100,381, 
1100,383, 
1100,395, 
1100,414, 
1100,433, 
1100,446, 
1100,441, 
1100,471, 
1100,480, 
1100,506, 
1100,503, 
1100,497, 
1100,506, 
1100,498, 
1100,499, 
1100,507, 
1100,471, 
1100,494, 
1100,439, 
1100,436, 
1100,410, 
1100,397, 
1100,398, 
1100,368, 
1100,354, 
1100,325, 
1100,319, 
1100,298, 
1100,284, 
1100,279, 
1100,267, 
1100,244, 
1100,241, 
1100,217, 
1100,209, 
1100,196, 
1100,188, 
1100,177, 
1100,165, 
1100,160, 
1100,151, 
1100,146, 
1100,133, 
1100,121, 
1100,115, 
1100,107, 
1100,102, 
1100,99, 
1100,95, 
1100,87, 
1100,92, 
1100,91, 
1100,87, 
1100,88, 
1100,86, 
1100,83, 
1100,85, 
1100,88, 
1100,86, 
1100,97, 
1100,91, 
1100,92, 
1100,98, 
1100,103, 
1100,109, 
1100,116, 
1100,127, 
1100,124, 
1100,166]
"""

data = [330,488, 
328,404, 
338,328, 
324,304, 
321,286, 
316,277, 
293,267, 
291,246, 
277,232, 
264,210, 
247,189, 
226,176, 
211,166, 
196,143, 
183,134, 
166,124, 
153,114, 
133,107, 
116,96, 
107,90, 
91,84, 
80,77, 
74,69, 
68,68, 
58,69, 
62,64, 
58,59, 
61,63, 
61,66, 
71,72, 
86,89, 
101,105, 
121,123, 
151,150, 
179,174, 
215,207, 
258,236, 
301,294, 
360,361, 
389,474, 
421,563, 
429,636, 
451,621, 
449,606]

main()