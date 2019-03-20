import numpy as np
from numpy import cos,sin
import argparse


class ForwardKinematicsHandler:
    def __init__(self, initial, link_translation, rotational_axes, joint_cuboid_positions, joint_cuboid_orientations):
        self.num_joints = initial.shape[0]
        if self.num_joints != link_translation.shape[0] or self.num_joints != rotational_axes.shape[0]:
            print("Error (Forward Kinematics): num_joints does not match!")

        self.initial = initial
        self.joint_transformations = []
        self.rotational_axes = rotational_axes

        for i in range(self.num_joints):
            rpy_list = np.array([0.0, 0.0, 0.0])
            H = self.constructTransformationMatrix(self.constructRotationMatrix(rpy_list), link_translation[i])

            self.joint_transformations.append(H)

        self.link_transformations = self.getLinkRotations(initial)
        self.transformations = []

        for i in range(self.num_joints):
            link_pose_H = self.constructTransformationMatrix(self.constructRotationMatrix(joint_cuboid_orientations[i]), joint_cuboid_positions[i])
            self.transformations.append(np.linalg.inv(self.link_transformations[i]) @ link_pose_H)
        # fingers
        finger_r_H = self.constructTransformationMatrix(self.constructRotationMatrix(joint_cuboid_orientations[5]), joint_cuboid_positions[5])
        finger_l_H = self.constructTransformationMatrix(self.constructRotationMatrix(joint_cuboid_orientations[6]), joint_cuboid_positions[6])
        self.transformations.append(np.linalg.inv(self.link_transformations[4]) @ finger_l_H)
        self.transformations.append(np.linalg.inv(self.link_transformations[4]) @ finger_r_H)

        # print("Transformations: ")
        # for t in self.transformations:
        #     print(t)
        #     print()

    def getLinkRotations(self, joint_angle_list):
        link_rotation = np.multiply(self.rotational_axes, np.tile(np.array(joint_angle_list).reshape(self.num_joints, 1), (1, 3)))
        H_list = []

        for i in range(self.num_joints):
            H = self.constructTransformationMatrix(self.constructRotationMatrix(link_rotation[i]), np.array([0.0, 0.0, 0.0]))
            if i == 0:
                H_list.append(self.joint_transformations[i] @ H)
            else:
                H_list.append(H_list[i-1] @ self.joint_transformations[i] @ H)
        return H_list


    def constructRotationMatrix(self, rpy_list):

        r = rpy_list[0]
        p = rpy_list[1]
        y = rpy_list[2]

        Rx = np.array([[1.0, 0.0, 0.0],
                      [0.0, cos(r), -sin(r)],
                      [0.0, sin(r), cos(r)]])

        Ry = np.array([[cos(p), 0.0, sin(p)],
                      [0.0, 1.0, 0.0],
                      [-sin(p), 0.0, cos(p)]])

        Rz = np.array([[cos(y), -sin(y), 0.0],
                      [sin(y), cos(y), 0.0],
                      [0.0, 0.0, 1.0]])

        return Rx @ Ry @ Rz

    def constructTransformationMatrix(self, R, t):

        t = np.array(t).reshape(3, 1)

        Rt = np.append(R, t, axis=1)
        H = np.append(Rt, np.array([0, 0, 0, 1]).reshape(1,-1), axis=0)

        return H

    def updateCuboidPoses(self, joint_angle_list):
        cuboid_poses = []
        self.link_transformations = self.getLinkRotations(joint_angle_list)

        for i in range(self.num_joints):
            cuboid_poses.append(self.link_transformations[i] @ self.transformations[i])
        # fingers
        cuboid_poses.append(self.link_transformations[4] @ self.transformations[5])
        cuboid_poses.append(self.link_transformations[4] @ self.transformations[6])

        return cuboid_poses