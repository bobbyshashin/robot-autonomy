# Import system libraries
import argparse
import os
import sys

# Modify the following lines if you have problems importing the V-REP utilities
cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(os.path.join(cwd,'lib'))
sys.path.append(os.path.join(cwd,'utilities'))

import numpy as np
from numpy import cos,sin
# import vrep_utils as vu

def constructRotationMatrix(orientation):
    r, p, y = orientation

    Rx = np.array([[1.0, 0.0, 0.0],
                  [0.0, cos(r), -sin(r)],
                  [0.0, sin(r), cos(r)]])

    Ry = np.array([[cos(p), 0.0, sin(p)],
                  [0.0, 1.0, 0.0],
                  [-sin(p), 0.0, cos(p)]])

    Rz = np.array([[cos(y), -sin(y), 0.0],
                  [sin(y), cos(y), 0.0],
                  [0.0, 0.0, 1.0]])

    return Rz @ Ry @ Rx

def constructVertices(dimension):
    dx, dy, dz = dimension
    vertices_constructor_matrix = np.array([[ 1,  1,  1],
                                            [ 1,  1, -1],
                                            [ 1, -1,  1],
                                            [-1,  1,  1],
                                            [ 1, -1, -1],
                                            [-1,  1, -1],
                                            [-1, -1,  1],
                                            [-1, -1, -1]]).T

    dim = np.array([dx * 0.5, dy * 0.5, dz * 0.5]).reshape(3, 1)

    return (vertices_constructor_matrix * np.tile(dim, (1, 8))).transpose()


class Cuboid:
    def __init__(self, origin, orientation, dimension):

        self.origin = np.array([origin[0], origin[1], origin[2]])

        self.rpy = np.array([orientation[0], orientation[1], orientation[2]])
        self.R = constructRotationMatrix(orientation)

        self.dimension = np.array([dimension[0], dimension[1], dimension[2]])
        self.vertices = constructVertices(self.dimension) @ self.R + self.origin # 8x3 matrix, each row is the [x, y, z] for one vertex



def constructCuboid(origin, orientation, dimension):

    cuboid = Cuboid(origin, orientation, dimension)
    return cuboid


def singleAxisCollisionCheck(axis, c1, c2):
    if c1.vertices.shape[0] != c2.vertices.shape[0]:
        print("Error: num of vertices does not match!")
        return None

    num_vertices = c1.vertices.shape[0]

    v1 = c1.vertices
    # print(v1)
    v2 = c2.vertices
    # print(v1.shape)
    # print()
    max1 = float('-inf')
    min1 = float('inf')
    max2 = float('-inf')
    min2 = float('inf')

    for i in range(num_vertices):

        dist1 = v1[i] @ axis
        # print("v1[i]: ", v1[i])
        # print("axis: ", axis)
        # print("dist1: ", dist1)
        dist2 = v2[i] @ axis
        # print("v2[i]: ", v2[i])
        # print("axis: ", axis)
        # print("dist2: ", dist2)
        if dist1 > max1:
            max1 = dist1
        if dist1 < min1:
            min1 = dist1
        if dist2 > max2:
            max2 = dist2
        if dist2 < min2:
            min2 = dist2

    # print("max1: ", max1)
    # print("min1: ", min1)
    # print("max2: ", max2)
    # print("min2: ", min2)

    longSpan = max(max1, max2) - min(min1, min2)
    sumSpan = max1 - min1 + max2 - min2
    # print(longSpan)
    # print(sumSpan)
    return longSpan <= sumSpan


def checkOverlap(max1, min1, max2, min2):

    if min1 > max2 or max1 < min2:
        return False
    else:
        return True


def findNormalVectors(c1, c2):

    # columns are x, y, z of the cuboid in world frame
    c1_axes = np.eye(3) @ c1.R
    c2_axes = np.eye(3) @ c2.R

    # each column is an axis to check SAT
    normals = np.empty((3, 0))

    for i in range(3):
        for j in range(3):
            new_axis = np.cross(c1_axes[:, i], c2_axes[:, j]).reshape(3, 1)
            if np.linalg.norm(new_axis) != 0.0:
                normals = np.append(normals, new_axis, axis=1)

    normals = np.append(normals, c1_axes, axis=1)
    normals = np.append(normals, c2_axes, axis=1)

    # for i in range(15):
    #     print(normals[:,i])
    return normals # 3xn matrix, each column is a normal vector (either of one surface of a cuboid, or of a pair of edges of two cuboids)

def collisionCheck(c1, c2):

    axes_to_check = findNormalVectors(c1, c2)
    # for row in axes_to_check.T:
    #     print(row)

    collide = True
    for i in range(axes_to_check.shape[1]):
        # print('i= ', i)
        # print("Axis: ", axes_to_check[:, i])
        yes = singleAxisCollisionCheck(axes_to_check[:, i], c1, c2)
        # print(yes)
        collide = collide and yes
    # print(collide)
    return collide


def vecProjection(vec_ref, vec):
    return (np.dot(vec_ref, vec) / (np.linalg.norm(vec_ref) ** 2)) * vec_ref


c_ref = Cuboid(origin=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0], dimension=[3.0, 1.0, 2.0])

c1 = Cuboid(origin=[0.0, 1.0, 0.0], orientation=[0.0, 0.0, 0.0], dimension=[0.8, 0.8, 0.8])
c2 = Cuboid(origin=[1.5, -1.5, 0.0], orientation=[1.0, 0.0, 1.5], dimension=[1.0, 3.0, 3.0])
c3 = Cuboid(origin=[0.0, 0.0, -1.0], orientation=[0.0, 0.0, 0.0], dimension=[2.0, 3.0, 1.0])
c4 = Cuboid(origin=[3.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0], dimension=[3.0, 1.0, 1.0])
c5 = Cuboid(origin=[-1.0, 0.0, -2.0], orientation=[0.5, 0.0, 0.4], dimension=[2.0, 0.7, 2.0])
c6 = Cuboid(origin=[1.8, 0.5, 1.5], orientation=[-0.2, 0.5, 0.0], dimension=[1.0, 3.0, 1.0])
c7 = Cuboid(origin=[0.0, -1.2, 0.4], orientation=[0.0, 0.785, 0.785], dimension=[1.0, 1.0, 1.0])
c8 = Cuboid(origin=[-0.8, 0.0, -0.5], orientation=[0.0, 0.0, 0.2], dimension=[1.0, 0.5, 0.5])


# c_test = Cuboid(origin=[-1.5, 1.5, 0.0], orientation=[0.0, 0.0, 0], dimension=[1.0, 2.9, 3.0])


# axes = np.eye(3) @ c_test.R
# print(axes)
# columns are the x, y, z axes in world frame, after rotation by R
# collisionCheck(c2, c_test)

# collisionCheck(c_ref, c1)
# collisionCheck(c_ref, c2)
# collisionCheck(c_ref, c3)
# collisionCheck(c_ref, c4)
# collisionCheck(c_ref, c5)
# collisionCheck(c_ref, c6)
# collisionCheck(c_ref, c7)
# collisionCheck(c_ref, c8)


'''
    debug
'''


# vertices_constructor_matrix = np.array([[ 1,  1,  1],
#                                         [ 1,  1, -1],
#                                         [ 1, -1,  1],
#                                         [-1,  1,  1],
#                                         [ 1, -1, -1],
#                                         [-1,  1, -1],
#                                         [-1, -1,  1],
#                                         [-1, -1, -1]]).T
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# lines = [(0, 1), (0, 2), (0, 3), (7, 6), (7, 5), (7, 4), (1, 5), (1, 6), (2, 4), (2, 7), (5,7), (4,7)]

# v1 = c_ref.vertices
# v2 = c6.vertices
# fig = plt.figure(1)
# ax = fig.add_subplot(111, projection='3d')

# for line in lines:
#     coords = np.stack([v1[line[0], :], v1[line[1], :]])
#     ax.plot(coords[:, 0], coords[:, 1], coords[:, 2], color='r', linewidth=2)

#     coords = np.stack([v1[line[1], :], v1[line[0], :]])
#     ax.plot(coords[:, 0], coords[:, 1], coords[:, 2], color='r', linewidth=2)

# for line in lines:
#     coords = np.stack([v2[line[0], :], v2[line[1], :]])
#     ax.plot(coords[:, 0], coords[:, 1], coords[:, 2], color='b', linewidth=2)
#     coords = np.stack([v2[line[1], :], v2[line[0], :]])
#     ax.plot(coords[:, 0], coords[:, 1], coords[:, 2], color='b', linewidth=2)


# ax.view_init(azim=-90., elev=90.)
# plt.show()