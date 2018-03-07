#rrt
import random as r
import math
import matplotlib.pyplot as pyplot

class Node:
    def __init__(self, coord):
        self.coord = coord
        self.next = []

    def printCoords(self):
    	print("coords: " + str(self.coord))

    def printNexts(self):
    	print("nexts: ")
    	for node in self.next:
    		print(node.coord)

    def equals(self, node):
    	return self.coord == node.coord

    def sizeOfNexts(self):
    	return len(self.next)

def radius(x, y):
	x2 = math.pow(x,2)
	y2 = math.pow(y,2)
	return math.sqrt(x2 + y2)

def bfsFindNear(head, qrand):
	queue = [head]
	currminDiff = 9999
	currNear = Node([0,0])
	while len(queue) > 0:
		curr = queue.pop(0)
		#print("size of curr.next is " + str(curr.sizeOfNexts()))
		nexts = curr.next
		for node in nexts:
			if node != None:
				queue.append(node)
		x = curr.coord[0]
		y = curr.coord[1]
		currRad = radius(x - qrand[0], y - qrand[1])
		#print("currRad is " + str(currRad))
		if currRad < currminDiff:
			currNear = curr
			currminDiff = currRad
	return currNear

def bfsFindAndSetNear(start, qnear, qnew):
	queue = [start]
	while len(queue) > 0:
		curr = queue.pop(0)
		nexts = curr.next
		for node in nexts:
			if node != None:
				queue.append(node)
		if curr.equals(qnear):
			curr.next.append(qnew)
			return True
	return False

def bfsFindIfPathExistTo(start, dest):
	queue = [(start, [(start, 20)])]
	destx = dest.coord[0]
	desty = dest.coord[1]
	i = 20
	while len(queue) > 0:
		curr = queue.pop(0)
		currNode = curr[0]
		currPath = curr[1]
		nexts = curr[0].next
		for node in nexts:
			if node != None:
				newPath = list(currPath)
				newPath.append((currNode, i))
				tup = (node, newPath)
				queue.append(tup)
		currx = curr[0].coord[0]
		curry = curr[0].coord[1]
		i += 20
		if abs(currx - destx) < ERR_RADIUS and abs(curry - desty) < ERR_RADIUS:
			#return path to curr
			return curr
	return None
	
def moveInc(qnear, qrand, dq):
	qrx = qrand[0]
	qry = qrand[1]
	mag = radius(qrx, qry)
	if mag != 0:
		qrxnorm = qrx/mag
		qrynorm = qry/mag
		qnearValx = qnear.coord[0]
		qnearValy = qnear.coord[1]
		qnearValx += qrxnorm*dq
		qnearValy += qrynorm*dq
		return Node([qnearValx, qnearValy])	#returns qnew
	else:
		return qnear

def getDataRRT(head):
	data = {"x":[], "y":[]}
	queue = [head]
	while len(queue) > 0:
		curr = queue.pop(0)
		nexts = curr.next
		for node in nexts:
			if node != None:
				queue.append(node)
		data['x'].append(curr.coord[0])
		data['y'].append(curr.coord[1])
	return data

def graph(x, y, xd, yd, xp, yp, color):
	pyplot.scatter(x, y, s=8)
	pyplot.scatter(xd, yd, s=64, c='g')
	pyplot.scatter(100, 100, s=64, c='g')
	pyplot.scatter(xp, yp, c=color)
	pyplot.plot(xp, yp, c='c')
	pyplot.show()

def graph2(x, y):
	pyplot.scatter(x, y, s=8)
	pyplot.show()

def rrt(init):
	start = Node(init)
	qnear = Node([0,0])
	qnew = Node([0,0])
	for k in range(1, K):
		qrand = [r.randint(-1*MAX_X,MAX_X),r.randint(-1*MAX_Y,MAX_Y)]
		qnear = bfsFindNear(start, qrand)	#returns a Node
		qnew = moveInc(qnear, qrand, 5)
		if not bfsFindAndSetNear(start, qnear, qnew):
			print("couldn't find qnear")
	return start

################################################
##                  Main                      ##
################################################
K = 1000
MAX_X = 100
MAX_Y = 100
ERR_RADIUS = 4
start = [100,100]
end = [0,0]

top = rrt(start)		#makes RRT graph, returns head node
goal = Node(end)		#goal node
res = getDataRRT(top)	#gets xy coordinates, seperates them into arrays
x = res['x']			#x coordinates of RRT graph
y = res['y']			#y coordinates of RRT graph
#graph2(x, y)
dest = bfsFindIfPathExistTo(top, goal)	#determines if a path to goal exists
if dest != None:
	x = []
	y = []
	c = []
	res = getDataRRT(top)	#gets xy coordinates for a short path from head to goal node
	for node in dest[1]:
		x.append(node[0].coord[0])	#x coordinates of shortest path graph
		y.append(node[0].coord[1])	#y coordinates of shortest path graph
		c.append(node[1])			#colors each node along path
	graph(res['x'], res['y'], end[0], end[1], x, y, c)
else:
	print("dest returned None, no path found.  Re-run or change some constant parameters")