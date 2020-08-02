# Import system libraries
import argparse
import os
import sys

import numpy as np
from math import fabs
import matplotlib.pyplot as plt

from PRM import PRM
from forward_kinematics import ForwardKinematicsHandler
from collision_checking import CollisionChecker, Cuboid, SATcollisionCheck
from arm_controller import ArmController

def main(args):


    deg_to_rad = np.pi/180.

    initial = np.array([    0.*deg_to_rad,
                            0.*deg_to_rad,
                            0.*deg_to_rad,
                            0.*deg_to_rad,
                            0.*deg_to_rad])

    final = np.array([    135.*deg_to_rad,
                          20.*deg_to_rad,
                          50.*deg_to_rad,
                         -80.*deg_to_rad,
                           0.*deg_to_rad])

    link_translation = np.array([[-0.0896, 0.00039, 0.159],
                                 [0,      0,      0.04125],
                                 [0.05,   0,      0.2],
                                 [0.2002, 0,      0],
                                 [0.063,  0.0001, 0]])

    rotational_axes = np.array([[0,  0, 1],
                                [0,  1, 0],
                                [0,  1, 0],
                                [0,  1, 0],
                                [-1, 0, 0]])

    JOINT_LOWER_LIMIT = np.array([-1.57, -1.57, -1.57, -1.57, -1.57])
    JOINT_UPPER_LIMIT = np.array([1.57, 1.57, 1.57, 1.57, 1.57])
    # Get obstacle cuboids and joint cuboids

    # obstacle_cuboid_names = vu.OBSTACLE_CUBOID_NAMES
    # joint_cuboid_names = vu.JOINT_CUBOID_NAMES

    joint_cuboid_positions = list(np.loadtxt("link_cuboid_positions.txt"))
    joint_cuboid_orientations = list(np.loadtxt("link_cuboid_orientations.txt"))
    joint_cuboid_dimensions = list(np.loadtxt("link_cuboid_dimensions.txt"))

    obstacle_cuboid_positions = [[0.2, 0.2, 0.099]]
    obstacle_cuboid_orientations = [[0.0, 0.0, 0.0]]
    obstacle_cuboid_dimensions = [[0.098, 0.072, 0.198]]


    joint_cuboids = []

    for i in range(len(joint_cuboid_positions)):
        pos = joint_cuboid_positions[i]
        ori = joint_cuboid_orientations[i]
        dim = joint_cuboid_dimensions[i]
        c = Cuboid(pos, ori, dim, "XYZ")
        # print(c.vertices)
        joint_cuboids.append(c)

    obstacle_cuboids = []

    for i in range(len(obstacle_cuboid_positions)):
        pos = obstacle_cuboid_positions[i]
        ori = obstacle_cuboid_orientations[i]
        dim = obstacle_cuboid_dimensions[i]
        c = Cuboid(pos, ori, dim, "XYZ")
        # print(c.vertices)
        obstacle_cuboids.append(c)

    num_joints = len(joint_cuboids)
    num_obstacles = len(obstacle_cuboids)

    foward_kinematics_handler = ForwardKinematicsHandler(initial, link_translation, rotational_axes, joint_cuboid_positions[1:], joint_cuboid_orientations[1:]) # TODO
    collision_checker = CollisionChecker(joint_cuboids, obstacle_cuboids, foward_kinematics_handler)    

    PRM_planner = PRM(JOINT_LOWER_LIMIT, JOINT_UPPER_LIMIT, 5, collision_checker) # TODO

    # sample = PRM_planner.randomSample()
    # sample = final
    # print("Sample: ")
    # print(sample)
    # collision_checker.checkCollisionSample(sample)

    ''' ===== Uncomment below to generate a new path ====='''
    path = PRM_planner.plan(initial, final, N=50, k=10)
    path_array = np.array(path)
    np.savetxt("path.txt", path_array)

    # path = list(np.loadtxt("path_500_74s.txt"))

    fingers = np.array([-0.03, 0.03])

    # Iterate through target joint positions
    # for target in path:

    #     target = np.append(target, fingers)
    #     # Set new target position
    #     controller.set_target_joint_positions(target)

    #     steady_state_reached = False
    #     while not steady_state_reached:

    #         timestamp = vu.get_sim_time_seconds(clientID)
    #         print('Simulation time: {} sec'.format(timestamp))

    #         # Get current joint positions
    #         sensed_joint_positions = vu.get_arm_joint_positions(clientID)
    #         # Calculate commands
    #         commands = controller.calculate_commands_from_feedback(timestamp, sensed_joint_positions)
    #         # Send commands to V-REP
    #         vu.set_arm_joint_target_velocities(clientID, commands)
    #         # Print current joint positions (comment out if you'd like)
    #         # print(sensed_joint_positions)
    #         vu.step_sim(clientID, 1)
    #         # Determine if we've met the condition to move on to the next point
    #         steady_state_reached = controller.has_stably_converged_to_target(timestamp)

    # vu.stop_sim(clientID)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    args = parser.parse_args()
    main(args)
