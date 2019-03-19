import numpy as np
from collision_checking import *

def randomSample(lower_limit, upper_limit, dimension):
	sample = []
	for i in range(dimension):
		r = np.random.random_sample()
		one_sample = (upper_limit[i] - lower_limit[i]) * r + lower_limit[i]
		sample.append(one_sample)
	return sample

def PRM(N):

	roadmap = None

	


