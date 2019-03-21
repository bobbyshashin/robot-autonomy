# Import system libraries
import argparse
import os
import sys

# Modify the following lines if you have problems importing the V-REP utilities
cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(os.path.join(cwd,'lib'))
sys.path.append(os.path.join(cwd,'utilities'))

# Import application libraries
import numpy as np
import vrep_utils as vu

# Import any other libraries you might want to use ############################
# ...
from math import fabs
import matplotlib.pyplot as plt

from PRM import PRM
from forward_kinematics import ForwardKinematicsHandler
from collision_checking import CollisionChecker, Cuboid, SATcollisionCheck
from arm_controller import ArmController

def main(args):
    # Connect to V-REP
    print ('Connecting to V-REP...')
    clientID = vu.connect_to_vrep()
    print ('Connected.')

    deg_to_rad = np.pi/180.

    # Joint targets. Specify in radians for revolute joints and meters for prismatic joints.
    # The order of the targets are as follows:
    #   joint_1 / revolute  / arm_base_link <- shoulder_link
    #   joint_2 / revolute  / shoulder_link <- elbow_link
    #   joint_3 / revolute  / elbow_link    <- forearm_link
    #   joint_4 / revolute  / forearm_link  <- wrist_link
    #   joint_5 / revolute  / wrist_link    <- gripper_link

    # Below two finger joints will not be commanded to move
    #   joint_6 / prismatic / gripper_link  <- finger_r
    #   joint_7 / prismatic / gripper_link  <- finger_l

    initial = np.array([  -80.*deg_to_rad,
                            0.*deg_to_rad,
                            0.*deg_to_rad,
                            0.*deg_to_rad,
                            0.*deg_to_rad])

    final = np.array([    0.*deg_to_rad,
                          60.*deg_to_rad,
                         -75.*deg_to_rad,
                         -75.*deg_to_rad,
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

    obstacle_cuboid_names = vu.OBSTACLE_CUBOID_NAMES
    joint_cuboid_names = vu.JOINT_CUBOID_NAMES

    obstacle_cuboid_handles = vu.get_obstacle_cuboid_handles(clientID)
    joint_cuboid_handles = vu.get_joint_cuboid_handles(clientID)

    joint_cuboid_positions = vu.get_cuboid_positions(clientID, joint_cuboid_handles)
    joint_cuboid_orientations = vu.get_cuboid_orientations(clientID, joint_cuboid_handles)
    joint_cuboid_dimensions = vu.get_cuboid_dimensions(clientID, joint_cuboid_handles)

    obstacle_cuboid_positions = vu.get_cuboid_positions(clientID, obstacle_cuboid_handles)
    obstacle_cuboid_orientations = vu.get_cuboid_orientations(clientID, obstacle_cuboid_handles)
    obstacle_cuboid_dimensions = vu.get_cuboid_dimensions(clientID, obstacle_cuboid_handles)

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

    # print("JOINT CUBOID POSITIONS: ")
    # print()
    # for jcp in joint_cuboid_positions[1:]:
    #     print(jcp)
    # print("JOINT CUBOID ORIENTATIONS: ")
    # print()
    # for jco in joint_cuboid_orientations[1:]:
    #     print(jco)


    # Test collision between obstacle and joints

    # for i in range(num_joints):
    #     for j in range(num_obstacles):
    #         collide = SATcollisionCheck(joint_cuboids[i], obstacle_cuboids[j])
    #         if collide:
    #             print(joint_cuboid_names[i] + " and " + obstacle_cuboid_names[j] + ": Collided!")

    # Test collision between different joints (arm's self-collision check)

    # for i in range(num_joints):
    #     for j in range(2, num_joints - i - 1):
    #         collide = SATcollisionCheck(joint_cuboids[i], joint_cuboids[i+j])
    #         if collide:
    #             print(joint_cuboid_names[i] + " and " + joint_cuboid_names[i+j] + ":")
    #             print(collide)

    joint_positions = vu.get_arm_joint_poses(clientID)

    # # Instantiate controller
    controller = ArmController()

    foward_kinematics_handler = ForwardKinematicsHandler(initial, link_translation, rotational_axes, joint_cuboid_positions[1:], joint_cuboid_orientations[1:]) # TODO
    collision_checker = CollisionChecker(joint_cuboids, obstacle_cuboids, foward_kinematics_handler)


    PRM_planner = PRM(JOINT_LOWER_LIMIT, JOINT_UPPER_LIMIT, 5, collision_checker) # TODO

    # sample = PRM_planner.randomSample()
    # sample = final
    # print("Sample: ")
    # print(sample)
    # collision_checker.checkCollisionSample(sample)

    ''' ===== Uncomment below to generate a new path ====='''
    # path = PRM_planner.plan(initial, final, N=1000, k=20)
    # path_array = np.array(path)
    # np.savetxt("path.txt", path_array)

    path = list(np.loadtxt("path_500_74s.txt"))

    fingers = np.array([-0.03, 0.03])
    # Reset simulation in case something was running
    vu.reset_sim(clientID)
    # Initial control inputs are zero
    vu.set_arm_joint_target_velocities(clientID, np.zeros(vu.N_ARM_JOINTS))
    # Despite the name, this sets the maximum allowable joint force
    vu.set_arm_joint_forces(clientID, 50.*np.ones(vu.N_ARM_JOINTS))
    # One step to process the above settings
    vu.step_sim(clientID)

    # Iterate through target joint positions
    for target in path:

        target = np.append(target, fingers)
        # Set new target position
        controller.set_target_joint_positions(target)

        steady_state_reached = False
        while not steady_state_reached:

            timestamp = vu.get_sim_time_seconds(clientID)
            print('Simulation time: {} sec'.format(timestamp))

            # Get current joint positions
            sensed_joint_positions = vu.get_arm_joint_positions(clientID)
            # Calculate commands
            commands = controller.calculate_commands_from_feedback(timestamp, sensed_joint_positions)
            # Send commands to V-REP
            vu.set_arm_joint_target_velocities(clientID, commands)
            # Print current joint positions (comment out if you'd like)
            # print(sensed_joint_positions)
            vu.step_sim(clientID, 1)
            # Determine if we've met the condition to move on to the next point
            steady_state_reached = controller.has_stably_converged_to_target(timestamp)

    vu.stop_sim(clientID)

    # # Post simulation cleanup -- save results to a pickle, plot time histories, etc #####
    # # Fill this out here (optional) or in your own script 
    # # If you use a separate script, don't forget to include it in the deliverables
    # # ...

    # time_history = controller.history['timestamp']
    # joint_feedback_history = controller.history['joint_feedback']
    # joint_target_history = controller.history['joint_target']
    # ctrl_command_history = controller.history['ctrl_commands']

    # joint_feedback_history = np.array(joint_feedback_history)
    # joint_target_history = np.array(joint_target_history)
    # ctrl_command_history = np.array(ctrl_command_history)

    # plt.figure()
    # plt.plot(time_history, joint_feedback_history[:, 0], 'r', linewidth=2, label='actual')
    # plt.plot(time_history, joint_target_history[:, 0], 'b', linewidth=2, label='target')
    # plt.xlabel('Timestamp (s)')
    # plt.ylabel('Joint1 Angle (rad)')
    # plt.legend()   
    # plt.grid()


    # plt.figure()
    # plt.plot(time_history, joint_feedback_history[:, 1], 'r', linewidth=2, label='actual')
    # plt.plot(time_history, joint_target_history[:, 1], 'b', linewidth=2, label='target')
    # plt.xlabel('Timestamp (s)')
    # plt.ylabel('Joint2 Angle (rad)')
    # plt.legend()   
    # plt.grid()
    
    # plt.figure()
    # plt.plot(time_history, joint_feedback_history[:, 2], 'r', linewidth=2, label='actual')
    # plt.plot(time_history, joint_target_history[:, 2], 'b', linewidth=2, label='target')
    # plt.xlabel('Timestamp (s)')
    # plt.ylabel('Joint3 Angle (rad)')
    # plt.legend()   
    # plt.grid()

    # plt.figure()
    # plt.plot(time_history, joint_feedback_history[:, 3], 'r', linewidth=2, label='actual')
    # plt.plot(time_history, joint_target_history[:, 3], 'b', linewidth=2, label='target')
    # plt.xlabel('Timestamp (s)')
    # plt.ylabel('Joint4 Angle (rad)')
    # plt.legend()   
    # plt.grid()

    # plt.figure()
    # plt.plot(time_history, joint_feedback_history[:, 4], 'r', linewidth=2, label='actual')
    # plt.plot(time_history, joint_target_history[:, 4], 'b', linewidth=2, label='target')
    # plt.xlabel('Timestamp (s)')
    # plt.ylabel('Joint5 Angle (rad)')
    # plt.legend()   
    # plt.grid()

    # plt.figure()
    # plt.plot(time_history, joint_feedback_history[:, 5], 'r', linewidth=2, label='actual')
    # plt.plot(time_history, joint_target_history[:, 5], 'b', linewidth=2, label='target')
    # # plt.plot(time_history, ctrl_command_history[:, 5], 'g', linewidth=2, label='ctrl_command')
    # plt.xlabel('Timestamp (s)')
    # plt.ylabel('Joint6 Angle (m)')
    # plt.legend()   
    # plt.grid()

    # plt.figure()
    # plt.plot(time_history, joint_feedback_history[:, 6], 'r', linewidth=2, label='actual')
    # plt.plot(time_history, joint_target_history[:, 6], 'b', linewidth=2, label='target')
    # # plt.plot(time_history, ctrl_command_history[:, 6], 'g', linewidth=2, label='ctrl_command')
    # plt.xlabel('Timestamp (s)')
    # plt.ylabel('Joint7 Angle (m)')
    # plt.legend()   
    # plt.grid()

    # plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    args = parser.parse_args()
    main(args)



# def main():
#     # Connect to V-REP
#     print ('Connecting to V-REP...')
#     clientID = vu.connect_to_vrep()
#     print ('Connected.')

#     # Reset simulation in case something was running
#     vu.reset_sim(clientID)

#     CUBOID_NAMES = ['arm_base_link_joint_collision_cuboid',
#                     'shoulder_link_collision_cuboid',
#                     'elbow_link_collision_cuboid',
#                     'forearm_link_collision_cuboid',
#                     'wrist_link_collision_cuboid',
#                     'gripper_link_collision_cuboid',
#                     'finger_r_collision_cuboid',
#                     'finger_l_collision_cuboid']

#     cuboid_positions = vu.get_cuboid_positions(clientID)
#     cuboid_orientations = vu.get_cuboid_orientations(clientID)
#     cuboid_dimensions = vu.get_cuboid_dimensions(clientID)

#     joint_positions = vu.get_arm_joint_poses(clientID)
#     print(joint_positions)

#     H_world_base = cuboid_positions[0].copy()

#     # # print(cuboid_positions)
#     # for i in range(len(cuboid_positions)):
#     #     for j in range(3):
#     #         # print(cuboid_positions[i][j])
#     #         cuboid_positions[i][j] -= H_world_base[j]
#     #         # print(cuboid_positions[i][j], H_world_base[j])


#     print("========== Joints ==========")
#     for i in range(len(joint_positions)):
#         pos = joint_positions[i]
#         # print(CUBOID_NAMES[i] + str(pos))
#         print("joint" + str(i+1))
#         print("x: " + str(pos[0]))
#         print("y: " + str(pos[1]))
#         print("z: " + str(pos[2]))
#         print()


#     print("========== Positions ==========")
#     for i in range(len(cuboid_positions)):
#         pos = cuboid_positions[i]
#         # print(CUBOID_NAMES[i] + str(pos))
#         print(CUBOID_NAMES[i])
#         print("x: " + str(pos[0]))
#         print("y: " + str(pos[1]))
#         print("z: " + str(pos[2]))
#         print()

#     print("========== Orientations ==========")

#     for i in range(len(cuboid_orientations)):
#         ori = cuboid_orientations[i]
#         print(CUBOID_NAMES[i] + str(ori))

#     print("========== Dimensions ==========")
#     for i in range(len(cuboid_dimensions)):
#         dim = cuboid_dimensions[i]
#         print(CUBOID_NAMES[i] + str(dim))

# if __name__ == '__main__':
#     main();