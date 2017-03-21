import numpy as np
import GenericAStar as SNode

class BoardCreator:

    def __init__(self, filename):
        self.board = []
        # Reads in file, saves A and B values, and creates new nodes in a matrix
        with open(filename) as file:
            lines = file.readlines()
            self.gridHeight = len(lines)

            for x, lines in enumerate(lines):

                self.board.append([])
                self.gridLength = len(lines)
                for y, char in enumerate(lines):
                    if char == "\n":
                        pass

                    self.board[x].GenericSearchNode(SNode.Node(x, y, char))

                    if char == "A":
                        self.start = (x, y)
                    if char == "B":
                        self.goal = (x, y)
        print self.board.sizeof()

