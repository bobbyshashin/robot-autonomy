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

def getWristPose(joint_angle_list, link_translation, link_rotation):
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
    # TODO("Complete this")
    
    # Read from URDF file
    # link_translation = np.array([[0,      0,      0.072],
    #                         [0,      0,      0.04125],
    #                         [0.05,   0,      0.2],
    #                         [0.2002, 0,      0],
    #                         [0.063,  0.0001, 0]])

    # link_rotation = np.array([[0, 0, 1*joint_angle_list[0]],
    #                      [0, 1*joint_angle_list[1],0],
    #                      [0, 1*joint_angle_list[2],0],
    #                      [0, 1*joint_angle_list[3],0],
    #                      [-1*joint_angle_list[4],0,0]])
    num_joints = len(joint_angle_list)
    T_total = np.eye(4)

    for i in range(num_joints):
        T_total = np.dot(T_total, constructTransformationMatrix(constructRotationMatrix(link_rotation[i]), link_translation[i].reshape(3, 1)))

    # flatten to 1*16 column major
    return T_total.flatten('F')

def getWristJacobian(joint_angle_list, link_translation, link_rotation):
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
    # TODO("Complete this")
    # Read from URDF file
    # link_translation = np.array([[0,      0,      0.072],
    #                         [0,      0,      0.04125],
    #                         [0.05,   0,      0.2],
    #                         [0.2002, 0,      0],
    #                         [0.063,  0.0001, 0]])

    # link_rotation = np.array([[0, 0, 1*joint_angle_list[0]],
    #                      [0, 1*joint_angle_list[1],0],
    #                      [0, 1*joint_angle_list[2],0],
    #                      [0, 1*joint_angle_list[3],0],
    #                      [-1*joint_angle_list[4],0,0]])

    n_joints = len(joint_angle_list)

    Jv = np.empty((3, 0))
    Jw = np.empty((3, 0))

    R_0i = np.eye(3)

    for i in range(n_joints):
        if i >= 1:
            T_0i_flattened = getWristPose(joint_angle_list[0:i], link_translation[0:i], link_rotation[0:i])
            T_0i = T_0i_flattened.reshape(4, 4).T
            R_0i = T_0i[0:3, 0:3]
        
        Jw = np.append(Jw, np.dot(R_0i, np.array([0, 0, 1]).reshape(3, 1)), axis=1)
    
    print(Jw)

def main(args):
    # joint_angles = args.joints
    joint_angles = [0.727, 1.073, 0.694, -0.244, 1.302]
    assert len(joint_angles) == 5, "Incorrect number of joints specified."

    # TODO("Change this as required")
    link_translation = np.array([[0,      0,      0.072],
                            [0,      0,      0.04125],
                            [0.05,   0,      0.2],
                            [0.2002, 0,      0],
                            [0.063,  0.0001, 0]])

    link_rotation = np.array([[0, 0, 1*joint_angles[0]],
                         [0, 1*joint_angles[1],0],
                         [0, 1*joint_angles[2],0],
                         [0, 1*joint_angles[3],0],
                         [-1*joint_angles[4],0,0]])

    pose = getWristPose(joint_angles, link_translation, link_rotation)
    jacobian = getWristJacobian(joint_angles, link_translation, link_rotation)

    print("Wrist pose: {}".format(np.array_str(np.array(pose), precision=2)))
    print("Jacobian: {}".format(
        np.array_str(np.array(jacobian), precision=2)))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
            description='Get wrist pose using forward kinematics')
    parser.add_argument('--joints', type=float, nargs='+', required=True,
                        help='Joint angles to get wrist position for.')
    args = parser.parse_args()
    
    main(args)
