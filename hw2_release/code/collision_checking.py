# Import system libraries
import argparse
import os
import sys

# Modify the following lines if you have problems importing the V-REP utilities
cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(os.path.join(cwd,'lib'))
sys.path.append(os.path.join(cwd,'utilities'))

from numpy import cos,sin
import vrep_utils as vu

def constructRotationMatrix(orientation):
    r, p, y = orientation
    return np.array([[cos(r)*cos(p), cos(r)*sin(p)*sin(y) - sin(r)*cos(y), cos(r)*sin(p)*cos(y) + sin(r)*sin(y)],
                     [sin(r)*cos(p), sin(r)*sin(p)*sin(y) + cos(r)*cos(y), sin(r)*sin(p)*cos(y) - cos(r)*sin(y)],
                     [-sin(p), cos(p)*sin(y), cos(p)*cos(y)]])

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

    dim = np.array([self.dx * 0.5, self.dy * 0.5, self.dz * 0.5]).reshape(3, 1)

    return vertices_constructor_matrix * np.tile(dim, (1, 8))


class Cuboid:
    def __init__(self, origin, orientation, dimension):

        self.origin = np.array([origin[0], origin[1], origin[2]])

        self.rpy = np.array([orientation[0], orientation[1], orientation[2]])
        self.R = constructRotationMatrix(orientation)

        self.dimension = np.array([dimension[0], dimension[1], dimension[2]])
        self.vertices = np.dot(constructVertices(self.dimension), self.R) + self.origin # 3x8 matrix, each column is the [x, y, z] for one vertex



def constructCuboid(origin, orientation, dimension):

    cuboid = Cuboid(origin, orientation, dimension)

def singleAxisCollisionCheck(max1, min1, max2, min2):
    if min1 > max2 or max1 < min2:
        return False
    else:
        return True

def collisionCheck(c1, c2):

    x1_max = np.max(c1.vertices[0])
    x1_min = np.min(c1.vertices[0])
    y1_max = np.max(c1.vertices[1])
    y1_min = np.min(c1.vertices[1])
    z1_max = np.max(c1.vertices[2])
    z1_min = np.min(c1.vertices[2])

    x2_max = np.max(c2.vertices[0])
    x2_min = np.min(c2.vertices[0])
    y2_max = np.max(c2.vertices[1])
    y2_min = np.min(c2.vertices[1])
    z2_max = np.max(c2.vertices[2])
    z2_min = np.min(c2.vertices[2])

    return (singleAxisCollisionCheck(x1_max, x1_min, x2_max, x2_min) and singleAxisCollisionCheck(y1_max, y1_min, y2_max, y2_min) and singleAxisCollisionCheck(z1_max, z1_min, z2_max, z2_min))




def main():
    # Connect to V-REP
    print ('Connecting to V-REP...')
    clientID = vu.connect_to_vrep()
    print ('Connected.')

    # Reset simulation in case something was running
    vu.reset_sim(clientID)

    CUBOID_NAMES = ['arm_base_link_joint_collision_cuboid',
                    'shoulder_link_collision_cuboid',
                    'elbow_link_collision_cuboid',
                    'forearm_link_collision_cuboid',
                    'wrist_link_collision_cuboid',
                    'gripper_link_collision_cuboid',
                    'finger_r_collision_cuboid',
                    'finger_l_collision_cuboid']


    cuboid_positions = vu.get_cuboid_positions(clientID)
    cuboid_orientations = vu.get_cuboid_orientations(clientID)
    cuboid_dimensions = vu.get_cuboid_dimensions(clientID)


    print("========== Positions ==========")
    for i in range(len(cuboid_positions)):
        pos = cuboid_positions[i]
        print(CUBOID_NAMES[i] + str(pos))

    print("========== Orientations ==========")

    for i in range(len(cuboid_orientations)):
        ori = cuboid_orientations[i]
        print(CUBOID_NAMES[i] + str(ori))

    print("========== Dimensions ==========")
    for i in range(len(cuboid_dimensions)):
        dim = cuboid_dimensions[i]
        print(CUBOID_NAMES[i] + str(dim))

if __name__ == '__main__':
    main();