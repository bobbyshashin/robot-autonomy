import numpy as np
# from collision_checking import CollisionChecker
from scipy.spatial import cKDTree
from a_star import A_Star_PRM

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

    self.path = None

  def randomSample(self):
    # sample = []
    # for i in range(self.dimension):
    #   r = np.random.random_sample()
    #   one_sample = (self.upper_limit[i] - self.lower_limit[i]) * r + self.lower_limit[i]
    #   sample.append(one_sample)
    return np.random.random_sample((5, )) * (self.upper_limit - self.lower_limit) + self.lower_limit

    # return sample # list of size "dimension"

  def uniformSample(self, N):
    return np.random.uniform(self.lower_limit, self.upper_limit, N)

  def interpolate(self, sample1, sample2, num_interval=3):
    p1 = np.array(sample1)
    p2 = np.array(sample2)

    p_diff = (p2 - p1) * 1.0 / num_interval

    self.path.append(p1)
    for i in range(num_interval):
        self.path.append(p1 + p_diff * i)

  def buildRoadmap(self, start, goal, num_samples, k):

    print("Start to build roadmap...")
    self.samples = []

    self.samples.append(start)
    self.samples.append(goal)

    while len(self.samples) != num_samples + 2:
      s = self.randomSample()
      # TODO: resample globally or around current sample? (globally for now)
      if self.collision_checker.checkCollisionSample(s) == False: # no collision
        self.samples.append(s)
        print(len(self.samples))
      else:
        s_new = np.random.normal(s, scale=0.1)
        while self.collision_checker.checkCollisionSample(s_new) == False:
            s_new = np.random.normal(s, scale=0.1)
            print("Gaussian sampling...")
        self.samples.append(s_new)
        print(len(self.samples))


    self.kd_tree = KD_Tree(self.samples)

    self.roadmap = []
    for i in range(len(self.samples)):
        edges = []
        distances, indices, neighbours = self.kd_tree.query(self.samples[i], k+1) # k+1 because a node consider itself as its nearest neighbour as well
        print(indices)
        for index in indices:
            if index != i:
                edges.append(index)

        self.roadmap.append(edges)

  def plan(self, start, goal, N, k):

    num_samples = N
    self.buildRoadmap(start, goal, num_samples, k)

    astar_planner = A_Star_PRM(start, goal, self.roadmap, self.samples, self.collision_checker)
    path = None
    if astar_planner.search():
        path = astar_planner.findPath()
        path.reverse()
        path.append(1) # goal
        print("Print path:")
        print(path)

    self.path = []
    for i in range(len(path) - 1):
        self.interpolate(self.samples[path[i]], self.samples[path[i+1]], 50)
    return self.path


