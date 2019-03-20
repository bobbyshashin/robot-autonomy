import numpy as np
from collision_checking import *
import scipy.spatial.cKDTree

JOINT_LOWER_LIMIT = np.array([-1.57, -1.57, -1.57, -1.57, -1.57])
JOINT_UPPER_LIMIT = np.array([1.57, 1.57, 1.57, 1.57, 1.57])

# def randomSample(lower_limit, upper_limit, dimension):
# 	sample = []
# 	for i in range(dimension):
# 		r = np.random.random_sample()
# 		one_sample = (upper_limit[i] - lower_limit[i]) * r + lower_limit[i]
# 		sample.append(one_sample)
# 	return sample
class KD_Tree:
    def __init__(self, data):
      self.kd_tree = cKDTree(data)
    def query(self)
# class KDTree:
#     """
#     Nearest neighbor search class with KDTree
#     """
#     def __init__(self, data):
#         # store kd-tree
#         self.tree = ss.cKDTree(data)

#     def NN_search(self, node, k=1):
#         """
#         Nearest Neighbour search for the input data
#         """
#         dist, index = self.tree.query(node, k=k)
#         neighbour = self.tree.data[index,:]
#         return index, dist, neighbour

#     def query(self, node, k):
#         distances, indexs = self.tree.query(node, k)
#         return indexs, distances

#     def NN_distance(self, node, r):
#         """
#         find points with in a distance r
#         """
#         index = self.tree.query_ball_point(node, r)
#         neighbours = self.tree.data[index,:]
#         return index, neighbours
    # def construct_roadmap(self, root, goal, N, k):
    #     print("Start generate roadmap!")
    #     ## input
    #     ## root : start configuration
    #     ## goal : goal configuration
    #     ## N : number of samples
    #     ## k : k nearest neighbours
    #     ## construct samples

    #     self.roadmap = []
    #     self.configs = []

    #     samples = np.zeros((N + 2, self.dimension))
    #     samples[0] = root
    #     samples[1] = goal
        
    #     for i in range(N):
    #         new_sample = self.uniform_sample()
    #         while not self.collision_check.single_query_check(new_sample):
    #             g_sample = np.random.multivariate_normal(new_sample, self.random_cov, 1)
    #             new_sample = g_sample[0]
    #         samples[i + 2] = new_sample
        
    #     ## get rid of duplicate samples
    #     # indexes = np.unique(samples, axis = 0, return_index=True)[1]
    #     # [samples[index] for index in sorted(indexes)]

    #     self.configs = samples

    #     ## construct KDTree
    #     kdtree = KDTree(samples)

    #     roadmap = []

    #     ## lazy PRM
    #     for s in samples:
    #         edges = []
    #         indexs, neighbours = kdtree.query(s, k)
    #         for index in indexs:
    #             edges.append(index)
                
    #         # for (index, neighbour) in zip(indexs, neighbours):
    #         #     if self.pass_collision_check(s, neighbour):
    #         #         edges.append(index)
        
    #         roadmap.append(edges)
    #     self.roadmap = roadmap
    #     return roadmap
class PRM:
  def __init__(self, lower_limit, upper_limit, dimension):
    self.lower_limit = lower_limit
    self.upper_limit = upper_limit
    self.dimension = dimension

    self.roadmap = None
    self.samples = None

  def randomSample(self):
    sample = []
    for i in range(self.dimension):
      r = np.random.random_sample()
      one_sample = (self.upper_limit[i] - self.lower_limit[i]) * r + self.lower_limit[i]
      sample.append(one_sample)
    return sample # list of size "dimension"

  def buildRoadmap(self, num_samples):

    self.samples = []

    while len(self.samples) != num_samples:
      s = randomSample()
      if noCollision(s): # TODO
        self.samples.append(s)


    self.kd_tree = KD_Tree(self.samples)

  def searchPath(self):

	


