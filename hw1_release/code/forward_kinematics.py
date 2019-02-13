import numpy as np
import argparse

def getWristPose(joint_angle_list, **kwargs):
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
    pass

def getWristJacobian(joint_angle_list, **kwargs):
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
    pass

def main(args):
    joint_angles = args.joints
    assert len(joint_angles) == 5, "Incorrect number of joints specified."

    # TODO("Change this as required")
    pose = getWristPose(joint_angles)
    jacobian = getWristJacobian(joint_angles)

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
