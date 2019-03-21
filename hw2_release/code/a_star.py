import numpy as np
from copy import deepcopy

class Node:
    def __init__(self, parent=-1, profile=None, index=-1, g=0.0, h=0.0, f=0.0):
        self.parent = parent
        self.profile = profile

        self.g = g
        self.h = h
        self.f = f

        self.index = index


# class PriorityQueue: 
#     def __init__(self): 
#         self.queue = [] 
  
#     def __str__(self): 
#         return ' '.join([str(i) for i in self.queue]) 
  
#     def isEmpty(self): 
#         return len(self.queue) == [] 

#     def insert(self, node): 
#         self.queue.append(deepcopy(node)) 
  
#     def pop(self): 
#         try: 
#             min_id = 0
#             for i in range(len(self.queue)): 
#                 if self.queue[i].f < self.queue[min_id].f: 
#                     min_id = i 
#             item = deepcopy(self.queue[min_id]) 
#             del self.queue[min_id] 
#             return item 
#         except IndexError: 
#             print() 
#             exit()
#     def clear(self):
#         self.queue.clear()
#     def exist(self, index):
#         for node in self.queue:
#             if node.index == index:
#                 return True
#         return False


class A_Star_PRM:
    def __init__(self, start, goal, roadmap, samples, collision_checker):
        self.start = Node(-1, start, 0)
        self.goal = Node(-1, goal, 1)

        self.open_list = {}
        self.closed_list = {}

        self.roadmap = roadmap
        self.samples = samples
        self.collision_checker = collision_checker

        self.path = None

    def euclidean_distance(self, node1, node2):
        return np.linalg.norm(np.array(node1.profile) - np.array(node2.profile))

    def heuristic(self, node):
        return self.euclidean_distance(node, self.goal)

    def findSmallestF(self):
        index = min(self.open_list, key=lambda node: self.open_list[node].f)
        return index

    def search(self):
        print("A* Search Begins...")
        self.open_list.clear()
        self.closed_list.clear()

        self.open_list[0] = self.start
        print(self.roadmap[0])
        print(self.roadmap[1])
        while True:
            if not self.open_list:
                print("Cannot find path, failed.")
                return False

            min_id = self.findSmallestF()
            print("Current node id: " + str(min_id))
            curr_node = self.open_list[min_id]
            self.closed_list[min_id] = deepcopy(curr_node)

            del self.open_list[min_id]

            if min_id == 1: # find goal
                print("Find goal!")
                self.goal.parent = curr_node.parent
                self.goal.g = curr_node.g
                break

            edges = self.roadmap[min_id]
            for e in edges:
                # if e == 1 and min_id != 0:
                #     print("Find goal!")
                #     self.goal.parent = min_id
                #     return True

                if e == min_id or e in self.closed_list: # pass nodes already visited
                    continue

                if e in self.open_list: # check for lower cost
                    next_node = self.open_list[e]
                    if self.collision_checker.validEdgeCheck(curr_node.profile, next_node.profile):
                        new_g = curr_node.g + self.euclidean_distance(curr_node, next_node)
                        if new_g < next_node.g: # cheaper cost, update this node
                            self.open_list[e].g = new_g
                            self.open_list[e].parent = min_id
                            self.open_list[e].f = new_g + next_node.h

                else: # new node
                    if self.collision_checker.validEdgeCheck(curr_node.profile, self.samples[e]):
                        new_node = Node(min_id, self.samples[e], e)
                        new_node.g = curr_node.g + self.euclidean_distance(curr_node, new_node)
                        new_node.h = self.heuristic(new_node)
                        new_node.f = new_node.g + new_node.h
                        self.open_list[e] = new_node
        return True

    def findPath(self):
        path = []
        parent = self.goal.parent
        while parent != -1:
            path.append(parent)
            parent = self.closed_list[parent].parent
        self.path = path
        return path
