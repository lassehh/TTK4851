import numpy as np
from sys import stdout
import math

#Assumes usage of the SearchNode-class in SearchNode.class

class Map:
	start = None
	goal = None
	gridHeight = None
	gridLength = None
	map = None

	#replace this init function with a function that loads an existing np.array instead,
	#and then creates a new one with pixel objects instead of boolean values
	def __init__(self, filename):
		with open(filename) as file:
			xSlices = file.readlines()
			self.gridHeight = len(xSlices)
			self.gridLength = len(xSlices[0]) - 1
			self.map = np.zeros(shape=(self.gridLength, self.gridHeight), dtype=np.object)
			for y, xSlice in enumerate(xSlices):
				#print self.gridHeight, self.gridLength
				for x, val in enumerate(xSlice):
					if x > self.gridLength - 1:
						break
					self.map[x,y] = Pixel(x, y, val)
					if val == "A":
						self.start = (x, y)
					if val == "B":
						self.goal = (x, y)

	def printMap(self):
		for y in range(0, self.gridHeight):
			for x in range(0, self.gridLength):
				stdout.write(self.map[x, y].value)
			print '\n'




	def closenessCheck(self, c):
		#to prevent generating a path too close to the boundaries of the objects
		#i.e: check nodes in a distance around the robot
		raise NotImplementedError

class Pixel:
	x = None									#State of the node, should be unique to that node(e-g- coordinates in a grid)
	y = None
	state = None
	parent = None								#the best parent node of this node
	kids = None									#all successors of this node
	g = None								#ard-cost
	f = None
	h = None
	value = None

	def __init__(self, x, y, val):
		self.x = x
		self.y = y
		self.state = [x,y]
		self.f = None
		self.h = None
		self.value = val
		if val == '.':
			self.g = 1.0
		if val == '#':
			self.g = float('Inf')
		else:
			self.g = 1.0
		self.parent = None
		self.kids = []

	def is_goal(self, map):
		if self.x == map.goal(0) and self.y == map.goal(1):
			return True
		else:
			return False

	def calc_h(self, map):
		return math.fabs(self.x - map.goal.x) + math.fabs(self.y - map.goal.y)

	def calc_f(self, map):							#calculate f = g+h
		return self.calc_h(map) + self.arc_cost()

	def generate_all_successors(self, map):
		successors = []
		pixel_up = map[(self.x),(self.y + 1)]
		pixel_down = map[(self.x),(self.y - 1)]
		pixel_left = map[(self.x - 1), (self.y)]
		pixel_right = map[(self.x + 1), (self.y)]
		successors.append(pixel_up)
		successors.append(pixel_down)
		successors.append(pixel_left)
		successors.append(pixel_right)
		return successors

	def arc_cost(self, pixel):
		return pixel.g


class AStar:
	"""docstring for GenericAStar"""
	created_dict = None  # Dictionary containing all nodes created
	open_list = []  # Nodes in the queue not yet visited
	closed_list = []  # Visted nodes
	n0 = None
	start = None
	search_type = None
	map = None

	def __init__(self, map, search_type):  # Initialise A*
		self.start = map.map[map.start[0], map.start[1]]  # Sets the start state
		self.search_type = search_type  # Sets search type (for module 1)
		self.created_dict = {(self.start.state): self.start}  # place the startnode in the created dictionary
		self.open_list = []  # open list is empty
		self.closed_list = []  # closed list is empty
		self.map = map

	def best_first_search(self):  # Main method of A*
		n0 = self.start  # makes n0 the startnode

		n0.calc_h(self.map)  # heuristic for the startnode
		n0.g = 0  # g = 0 for the startnode
		n0.calc_f(self.map)  # h+g

		self.open_list.append(n0)  # places the startnode in the open list

		# Agenda Loop
		# while no solution
		while (self.open_list):  # Agenda
			x = self.search_queue_pop(self.search_type,
									  self.open_list)  # Node currently working on set to the chosen node in the open list
			self.closed_list.append(x)  # Puts the node in the closed list
			if x.is_goal(self.map):  # fix if check											#Returns the path if the current node is the goal
				return self.path(x)  # return the goal path
			successors = x.generate_all_successors(self.map)  # Generate all the succesors of the current node

			for s in successors:  # For each successor of the current node
				if (
				s.state) in self.created_dict:  # sets the current successor to the allready created node, if it exists
					s = self.created_dict[(s.state)]

				if s not in self.open_list and s not in self.closed_list:  # if not in open or closed list
					self.attach_and_eval(s, x, self.map)  # run attach and eval for that node and the current node
					self.open_list.append(s)  # add the node to the agenda queue(open list)
					self.created_dict[(s.state)] = s  # Places the new node in the created dictionary
				elif x.g + x.arc_cost(
						s) < s.g:  # else if g of the current node + the cost of x to s is less than the g value of current successor's g
					self.attach_and_eval(s, x)  # run attach and eval of s and x
					if s in self.closed_list:  # if s allraedy is in closed list
						self.propagate_path_improvements(s)  # run propagate_path_improvements

		return []

	def search_queue_pop(self, search_type, queue):  # for popping elements off the agenda queue (open list)
		if search_type == "BFS":  # Breadth First Search mode
			return queue.pop(0)
		elif search_type == "DFS":  # Depth First Search mode
			return queue.pop()
		elif search_type == "BestFS":  # Best First Search mode
			current_node = min(queue, key=lambda x: x.f)
			queue.remove(current_node)
			return current_node
		else:  # If not recognized search mode
			raise NotImplementedError

	def attach_and_eval(self, c, p, map):  # Sert parent, g, h and f values of a new node
		c.parent = p
		c.g = p.g + p.arc_cost(c)
		c.h = c.calc_h(map)
		c.f = c.g + c.h

	def propagate_path_improvements(self, p):  # Updates paretn, g, h and f values if a better parent is found
		for c in p.kids:
			if p.g + p.arc_cost(c) < c.g:
				c.parent = p
				c.g = p.g + p.arc_cost(c)
				c.f = c.g + c.h
				self.propagate_path_improvements(c)

	def path(self, x):  # Returns the path from the start node/state to the goal node/state
		goal_path = [x.state]
		while x.parent:
			x = x.parent
			goal_path.append(x.state)
		return goal_path[::-1]
