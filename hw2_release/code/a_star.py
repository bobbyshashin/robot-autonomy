import numpy as np

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0


class A_star:
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal

        self.open_list = {}
        self.closed_list = {}

    def distance(self, node1, node2):
        # euclidean distance
        return np.linalg.norm(node1.position - node2.position)

    def heuristic(self, node):
        return self.distance(node, self.goal)

    def search(self):
        self.open_list.clear()
        self.closed_list.clear()

        self.open_list.update(self.start)

        while True:
            if self.open_list == False:
                print("Cannot find path, failed.")
                return False

            curr = self.open_list[min(self.open_list, key=lambda node: self.open_list[node].f)]


class PRM:
    def __init__(self, lower_limit, upper_limit):
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit