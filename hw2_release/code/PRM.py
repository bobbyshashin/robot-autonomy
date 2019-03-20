import numpy as np
from collision_checking import *
from scipy.spatial import cKDTree

class KD_Tree: # for nearest neighbour search
    def __init__(self, data):
      self.kd_tree = cKDTree(data)

    def query(self, node, k):
    	dist, index = self.kd_tree.query(x=node, k=k)
    	neighbours = self.kd_tree.data[index, :]

    	return dist, index, neighbours

    def query_ball_point(self, node, r):
    	index = self.kd_tree.query_ball_point(x=node, r=r)
    	neighbours = self.kd_tree.data[index, :]

    	return index, neighbours

class PRM:
  def __init__(self, lower_limit, upper_limit, dimension, collision_checker):
    self.lower_limit = lower_limit
    self.upper_limit = upper_limit
    self.dimension = dimension

    self.collision_checker = collision_checker

    self.roadmap = None
    self.samples = None
    self.kd_tree = None

  def randomSample(self):
    sample = []
    for i in range(self.dimension):
      r = np.random.random_sample()
      one_sample = (self.upper_limit[i] - self.lower_limit[i]) * r + self.lower_limit[i]
      sample.append(one_sample)
    return sample # list of size "dimension"

  def buildRoadmap(self, num_samples, k):

    self.samples = []

    while len(self.samples) != num_samples:
      s = randomSample()
      if self.collision_checker.checkCollisionSingleSample(s): # TODO
        self.samples.append(s)


    self.kd_tree = KD_Tree(self.samples)

    for sample in self.samples:
    	edges = []
    	index, neighbours = self.kd_tree.query(sample, k)
    	for i in index:
    		edges.append(i)

    	self.roadmap.append(edges)

  def searchPath(self):
  	pass

	


