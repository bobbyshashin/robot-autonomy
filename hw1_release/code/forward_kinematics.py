import numpy as np
from numpy import cos,sin
import argparse

def constructRotationMatrix(rpy_list):
    r = rpy_list[2]
    p = rpy_list[1]
    y = rpy_list[0]
    return np.array([[cos(r)*cos(p), cos(r)*sin(p)*sin(y) - sin(r)*cos(y), cos(r)*sin(p)*cos(y) + sin(r)*sin(y)],
                     [sin(r)*cos(p), sin(r)*sin(p)*sin(y) + cos(r)*cos(y), sin(r)*sin(p)*cos(y) - cos(r)*sin(y)],
                     [-sin(p), cos(p)*sin(y), cos(p)*cos(y)]])

def constructTransformationMatrix(R, t):

    Rt = np.append(R, t, axis=1)
    T = np.append(Rt, np.array([0, 0, 0, 1]).reshape(1,-1), axis=0)

    return T

def getWristPose(joint_angle_list, rotational_axes, link_translation):
    '''Get the wrist pose for given joint angle configuration.

    joint_angle_list: List of joint angles to get final wrist pose for
    kwargs: Other keyword arguments use as required.

    TODO: You can change the signature of this method to pass in other objects,
    such as the path to the URDF file or a configuration of your URDF file that
    has been read previously into memory. 

    Return: List of 16 values which represent the joint wrist pose 
    obtained from the End-Effector Transformation matrix using column-major
    ordering.
    '''
    num_joints = len(joint_angle_list)
    link_rotation = np.multiply(rotational_axes, np.tile(np.array(joint_angle_list).reshape(num_joints, 1), (1, 3)))

    T_total = np.eye(4)

    for i in range(num_joints):
        T_total = np.dot(T_total, constructTransformationMatrix(constructRotationMatrix(link_rotation[i]), link_translation[i].reshape(3, 1)))

    # flatten to 1*16 column major
    return T_total.flatten('F')

def getWristJacobian(joint_angle_list, rotational_axes, link_translation):
    '''Get the wrist jacobian for given joint angle configuration.

    joint_angle_list: List of joint angles to get final wrist pose for
    kwargs: Other keyword arguments use as required.

    TODO: You can change the signature of this method to pass in other objects,
    such as the wrist pose for this configuration or path to the URDF
    file or a configuration of your URDF file that has been read previously
    into memory. 

    Return: List of 16 values which represent the joint wrist pose 
    obtained from the End-Effector Transformation matrix using column-major
    ordering.
    '''
    num_joints = len(joint_angle_list)

    link_rotation = np.multiply(rotational_axes, np.tile(np.array(joint_angle_list).reshape(num_joints, 1), (1, 3)))

    Jv = np.empty((3, 0))
    Jw = np.empty((3, 0))

    # R_0i = np.eye(3)
    T_0n = getWristPose(joint_angle_list, rotational_axes, link_translation).reshape(4,4).T
    t_0n = T_0n[0:3, 3]

    for i in range(num_joints):

        T_0i_flattened = getWristPose(joint_angle_list[0:i+1], rotational_axes[0:i+1], link_translation[0:i+1])
        T_0i = T_0i_flattened.reshape(4, 4).T

        R_0i = T_0i[0:3, 0:3]
        t_0i = T_0i[0:3, 3]

        Jv = np.append(Jv, np.cross(np.dot(R_0i, rotational_axes[i]).reshape(1, 3), (t_0n - t_0i).reshape(1, 3)).reshape(3, 1), axis=1)
        Jw = np.append(Jw, np.dot(R_0i, rotational_axes[i]).reshape(3, 1), axis=1)

    
    J = np.append(Jv, Jw, axis=0)
    return J

def main(args):
    joint_angles = args.joints
    assert len(joint_angles) == 5, "Incorrect number of joints specified."

    # TODO("Change this as required")
    link_translation = np.array([[0,      0,      0.072],
                                 [0,      0,      0.04125],
                                 [0.05,   0,      0.2],
                                 [0.2002, 0,      0],
                                 [0.063,  0.0001, 0]])

    rotational_axes = np.array([[0,  0, 1],
                                [0,  1, 0],
                                [0,  1, 0],
                                [0,  1, 0],
                                [-1, 0, 0]])

    pose = getWristPose(joint_angles, rotational_axes, link_translation)
    jacobian = getWristJacobian(joint_angles, rotational_axes, link_translation)

    print("Wrist pose: {}".format(np.array_str(np.array(pose), precision=3)))
    print("Jacobian: {}".format(np.array_str(np.array(jacobian), precision=3)))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
            description='Get wrist pose using forward kinematics')
    parser.add_argument('--joints', type=float, nargs='+', required=True,
                        help='Joint angles to get wrist position for.')
    args = parser.parse_args()
    
    main(args)
