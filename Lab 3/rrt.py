#rrt
import random as r
import math
import matplotlib.pyplot as pyplot

#each point in the graph will be a Node, which has:
	#a x-coordinate and y-coordinate stored in coord (coord is a list-type)([])
	#a list of next Nodes.  next is every Node that was considered closest to the current Node
class Node:
    def __init__(self, coord):
        self.coord = coord 		#list type
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

#radius or magnitude given x and y
def radius(x, y):
	x2 = math.pow(x,2)
	y2 = math.pow(y,2)
	return math.sqrt(x2 + y2)

#find the closest node to qrand
#keep the current closest distance as currminDiff
#keep the current closest Node as currNear
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

#we know a valid qnear exists, so find it and append qnew to qnear.next
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

#determine if a path exists from start to dest
def bfsFindIfPathExistTo(start, dest):
	queue = [(start, [(start, 20)])]
	#queue is an array of tuples.
	#the tuple's elements are a node, and an list of tuples that contain the path to current node
	#the inner tuple's elements are a node, and an integer that represents a color
	#the color is used in graphing to easily determine if other nodes of the same parent are connected
	#to summarize:
		#to get a node: queue[i][0] -> returns a Node object
			#to get current node: i = 0
		#to get a node's path: queue[i][1] -> returns a list
		#to get a node within a node's path: queue[i][1][k][0] -> returns a Node object
			#k is the k'th node in the path
		#to get a node within a node's path's color: queue[i][1][k][1] -> returns an int
	destx = dest.coord[0]
	desty = dest.coord[1]
	i = 20	#color
	while len(queue) > 0:
		curr = queue.pop(0)
		currNode = curr[0]
		currPath = curr[1]
		nexts = curr[0].next
		for node in nexts:
			if node != None:
				newPath = list(currPath)	#makes a copy of the list, instead of a pointer
				newPath.append((currNode, i))
				tup = (node, newPath)
				queue.append(tup)
		currx = curr[0].coord[0]
		curry = curr[0].coord[1]
		i += 20
		if abs(currx - destx) < ERR_RADIUS and abs(curry - desty) < ERR_RADIUS:
			return curr
	return None

#move in the direction of qrand with a stepsize of dq
def moveInc(qnear, qrand, dq, obstacles):
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
		if isValidMove([qnearValx, qnearValy], obstacles):
			return Node([qnearValx, qnearValy])	#returns qnew
		else:
			return None
	else:
		return None

#checks to see if moving to a point will intersect with an obstacle
def isValidMove(point, obstacles):
	for obstacle in obstacles:
		xo, yo = obstacle[0][0], obstacle[0][1]
		w, h = obstacle[1][0], obstacle[1][1]
		xp, yp = point[0], point[1]
		if (xp >= xo and xp <= xo + w) and (yp >= yo and yp <= yo + h):	#if inside an obstacle
			return False
	return True

#creates a list of lists for graphing obstacles
def listOfObstacles(obstacles):
	listOfObs = []
	for obstacle in obstacles:
		xpoints = []
		ypoints = []
		x, y = obstacle[0][0], obstacle[0][1]
		w, h = obstacle[1][0], obstacle[1][1]
		xpoints.append(x)
		ypoints.append(y)
		xpoints.append(x+w)
		ypoints.append(y)
		xpoints.append(x+w)
		ypoints.append(y+h)
		xpoints.append(x)
		ypoints.append(y+h)
		xpoints.append(x)
		ypoints.append(y)
		listOfObs.append((xpoints, ypoints))
	return listOfObs

#draw all obstacles as red boxes
def drawObstacle(obstacles):
	for obs in obstacles:
		pyplot.plot(obs[0], obs[1], c='r')

#separates an array of coordinates into seperate arrays of x and y coordinates for graphing purposes
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

#helper function to display graphs
def graph(x, y, xd, yd, xp, yp, color):
	pyplot.scatter(x, y, s=8)
	pyplot.scatter(xd, yd, s=64, c='g')
	pyplot.scatter(100, 100, s=64, c='g')
	pyplot.scatter(xp, yp, c=color)
	pyplot.plot(xp, yp, c='c')

#helper function to display graphs
def graph2(x, y, c='r'):
	pyplot.plot(x, y, c)
	pyplot.show()

#RRT algo, follows pseudocode on wikipedia for the most part
def rrt(init, obstacles):
	start = Node(init)
	qnear = Node([0,0])
	qnew = Node([0,0])
	for k in range(1, K):
		qrand = [r.randint(-1*MAX_X,MAX_X), r.randint(-1*MAX_Y,MAX_Y)]
		qnear = bfsFindNear(start, qrand)	#returns a Node
		qnew = moveInc(qnear, qrand, 5, obstacles)
		if qnew is None:
			continue
		if not bfsFindAndSetNear(start, qnear, qnew):
			print("couldn't find qnear")
	return start

################################################
##                  Main                      ##
################################################
K = 1000
MAX_X = 250
MAX_Y = 250
ERR_RADIUS = 5
start = [100,100]
end = [10,10]
#format [(bottom left corner x, y), (width, height)]
obstacles = [[(50,50), (25, 25)], [(25,25), (10, 10)]]

top = rrt(start, obstacles)		#makes RRT graph, returns head node
goal = Node(end)				#goal node
dest = bfsFindIfPathExistTo(top, goal)	#determines if a path to goal exists
while dest == None:
	top = rrt(start, obstacles)		#repeat until a path exists
	dest = bfsFindIfPathExistTo(top, goal)	#repeat
res = getDataRRT(top)			#gets xy coordinates, seperates them into arrays
x = res['x']					#x coordinates of RRT graph
y = res['y']					#y coordinates of RRT graph
if dest != None:
	x = []
	y = []
	c = []
	res = getDataRRT(top)		#gets xy coordinates for a short path from head to goal node
	for node in dest[1]:
		x.append(node[0].coord[0])	#x coordinates of shortest path graph
		y.append(node[0].coord[1])	#y coordinates of shortest path graph
		c.append(node[1])			#colors each node along path
	graph(res['x'], res['y'], end[0], end[1], x, y, c)
	obs = listOfObstacles(obstacles)
	drawObstacle(obs)
	pyplot.show()
else:
	print("dest returned None, no path found.  Re-run or change some constant parameters")