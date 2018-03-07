#rrt
import random as r
import math
import matplotlib.pyplot as pyplot

class Node:
    def __init__(self, coord):
        self.coord = coord
        self.next = []

    def getCoord(self):
        return self.coord

    def getNext(self):
        return self.next

    def setCoord(self,newdata):
        self.coord = newdata

    def setNext(self,newnext):
        self.next.append(newnext)

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


MAX_X = 100
MAX_Y = 100
ERR_RADIUS = 4

def radius(x, y):
	x2 = math.pow(x,2)
	y2 = math.pow(y,2)
	return math.sqrt(x2 + y2)

def bfsFindNear(head, qrand):
	#printRRT(head)
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
			curr.next.append(qnew)	#try setNext?
			return True
	return False

def bfsFindIfPathExistTo(start, dest):
	print("start is " + str(start))
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
				print("i is " + str(i))
				print("currNode is " + str(currNode.coord))
				newPath = list(currPath)
				newPath.append((currNode, i))
				print("curr path is ")
				for n in newPath:
					print(n[0].coord)
				tup = (node, newPath)
				queue.append(tup)
		currx = curr[0].coord[0]
		curry = curr[0].coord[1]
		i += 20
		if abs(currx - destx) < ERR_RADIUS and abs(curry - desty) < ERR_RADIUS:
			#return path to curr
			return curr
	return None

def bfsFindPath(start, dest):
	queue = [start]
	graph = {}
	graphcoord = {}
	destx = dest.coord[0]
	desty = dest.coord[1]
	while len(queue) > 0:
		curr = queue.pop(0)
		children = []
		nexts = curr.next
		for node in nexts:
			if node != None:
				queue.append(node)
				children.append(node)
		currx = curr.coord[0]
		curry = curr.coord[1]
		graph[curr] = set(children)
		graphcoord[(currx,curry)] = children
		if abs(currx - destx) < ERR_RADIUS and abs(curry - desty) < ERR_RADIUS:
			print(graphcoord)
			q = [(start, [start])]
			while q:
				(vertex, path) = q.pop(0)
				for next in graphcoord[vertex] - set(path):
					if next == dest:
						print("found path ")
						print(path)
						return path + [next]
					else:
						q.append((next, path + [next]))
	return None
	
def moveInc(qnear, qrand, dq):
	qrx = qrand[0]
	qry = qrand[1]
	mag = radius(qrx, qry)
	qrxnorm = qrx/mag
	qrynorm = qry/mag
	qnearValx = qnear.coord[0]
	qnearValy = qnear.coord[1]
	qnearValx += qrxnorm*dq
	qnearValy += qrynorm*dq
	return Node([qnearValx, qnearValy])	#returns qnew

#start = Node([0,0])
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

def printRRT(head):
	queue = [head]
	i = 0
	j = 0
	while len(queue) > 0:
		curr = queue.pop(0)
		nexts = curr.next
		for node in nexts:
			if node != None:
				queue.append(node)
				j += 1
		print(curr.coord)
		i += 1
		print("j is :" + str(j))
		j = 0
	#head.printNexts()
	print("i is :" + str(i))

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
	pyplot.scatter(xd, yd, s=32, c='y')
	pyplot.scatter(100, 100, s=32, c='g')
	pyplot.scatter(xp, yp, c=color)
	pyplot.plot(xp, yp, c='c')
	pyplot.show()

def graph2(x, y):
	pyplot.scatter(x, y, s=8)
	pyplot.show()


K = 1000
#printRRT(rrt([100,100]))
top = rrt([100,100])
goal = Node([0,0])
res = getDataRRT(top)
x = res['x']
y = res['y']
#graph2(x, y)
dest = bfsFindIfPathExistTo(top, goal)
if dest != None:
	print(dest[0])
	print(dest[1])
	x = []
	y = []
	c = []
	res = getDataRRT(top)
	for node in dest[1]:
		x.append(node[0].coord[0])
		y.append(node[0].coord[1])
		c.append(node[1])
	graph(res['x'], res['y'], 50, 50, x, y, c)
else:
	print("dest returned None")