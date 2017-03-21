import numpy as np
from sys import stdout
import math
import glob


class Domain:
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




	#def closenessCheck(self, c):
	#	#to prevent generating a path too close to the boundaries of the objects
	#	#i.e: check nodes in a distance around the robot
	#	raise NotImplementedError

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
		self.state = (x,y)
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