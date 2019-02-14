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

###############################################################################

class ArmController:

    def __init__(self):
        # Fill out this method ##################################
        # Define any variables you may need here for feedback control
        # ....

        # For revolute joints
        self.Kp = 10.0
        self.Ki = 0.01
        self.Kd = 0.1

        # For prismatic joints
        self.Kp_pris = 10.0
        self.Ki_pris = 0.01
        self.Kd_pris = 0.1


        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

        self.i_limit = 1.0

        self.last_timestamp = None
        self.last_error = None
        self.first_flag = True

        self.converged_time_interval = 1.5
        self.converged_time_buffer = 0.0
        self.time_flag = False

        #########################################################
        # Do not modify the following variables
        self.history = {'timestamp': [],
                        'joint_feedback': [],
                        'joint_target': [],
                        'ctrl_commands': []}
        self._target_joint_positions = None

    def set_target_joint_positions(self, target_joint_positions):
        assert len(target_joint_positions) == vu.N_ARM_JOINTS, \
            'Expected target joint positions to be length {}, but it was length {} instead.'.format(len(target_joint_positions), vu.N_ARM_JOINTS)
        self._target_joint_positions = target_joint_positions

    def calculate_commands_from_feedback(self, timestamp, sensed_joint_positions):
        assert self._target_joint_positions, \
            'Expected target joint positions to be set, but it was not.'

        # Fill out this method ##################################
        # Using the input joint feedback, and the known target joint positions,
        # calculate the joint commands necessary to drive the system towards
        # the target joint positions.
        ctrl_commands = np.zeros(vu.N_ARM_JOINTS)
        # ...

        error = np.array(self._target_joint_positions) - np.array(sensed_joint_positions)
        # print("Error: ", error)

        if self.first_flag == True:
            self.last_timestamp = timestamp
            self.last_error = error
            self.first_flag = False
            return ctrl_commands

        dt = timestamp - self.last_timestamp

        self.p_term = error
        self.i_term += dt * error

        for i in range(self.i_term.shape[0]):
            if self.i_term[i] <= -self.i_limit:
                self.i_term[i] = -self.i_limit
            elif self.i_term[i] >= self.i_limit:
                self.i_term[i] = self.i_limit

        self.d_term = (error - self.last_error) / dt


        self.last_timestamp = timestamp
        self.last_error = error

        ctrl_commands[0:-2] = self.Kp * self.p_term[0:-2] + self.Ki * self.i_term[0:-2] + self.Kd * self.d_term[0:-2]
        # different gains for prismatic joints
        ctrl_commands[-2:] = self.Kp_pris * self.p_term[-2:] + self.Ki_pris * self.i_term[-2:] + self.Kd_pris * self.d_term[-2:]

        # print("Ctrl commands: ", ctrl_commands)
        #########################################################

        # Do not modify the following variables
        # append time history
        self.history['timestamp'].append(timestamp)
        self.history['joint_feedback'].append(sensed_joint_positions)
        self.history['joint_target'].append(self._target_joint_positions)
        self.history['ctrl_commands'].append(ctrl_commands)
        return ctrl_commands

    def has_stably_converged_to_target(self, timestamp):
        # Fill out this method ##################################
        has_stably_converged_to_target = True
        num_joints = vu.N_ARM_JOINTS
        if (len(self._target_joint_positions) != num_joints) or (len(self.last_error) != num_joints):
            print("Error: dimension inconsistent!")

        for i in range(num_joints):
            # if fabs(self.last_error[i]) >= fabs(self._target_joint_positions[i] * 0.01): # within 1% range
            if i < num_joints - 2:
                if fabs(self.last_error[i]) >= 0.01:
                    has_stably_converged_to_target = False
            else:
                if fabs(self.last_error[i]) >= 0.002:
                    has_stably_converged_to_target = False
        # timestamp = vu.get_sim_time_seconds(clientID)
        if has_stably_converged_to_target == True and self.time_flag == False:
            self.converged_time_buffer = timestamp
            self.time_flag = True

        if timestamp - self.converged_time_buffer < self.converged_time_interval:
            has_stably_converged_to_target = False
        else:
            self.time_flag = False

        if has_stably_converged_to_target:
            self.i_term = 0.0


        # ...
        # if has_stably_converged_to_target == True:
        #     print("F**k yeah!")
        #########################################################
        return has_stably_converged_to_target


def main(args):
    # Connect to V-REP
    print ('Connecting to V-REP...')
    clientID = vu.connect_to_vrep()
    print ('Connected.')

    # Reset simulation in case something was running
    vu.reset_sim(clientID)
    
    # Initial control inputs are zero
    vu.set_arm_joint_target_velocities(clientID, np.zeros(vu.N_ARM_JOINTS))

    # Despite the name, this sets the maximum allowable joint force
    vu.set_arm_joint_forces(clientID, 50.*np.ones(vu.N_ARM_JOINTS))

    # One step to process the above settings
    vu.step_sim(clientID)

    deg_to_rad = np.pi/180.

    # Joint targets. Specify in radians for revolute joints and meters for prismatic joints.
    # The order of the targets are as follows:
    #   joint_1 / revolute  / arm_base_link <- shoulder_link
    #   joint_2 / revolute  / shoulder_link <- elbow_link
    #   joint_3 / revolute  / elbow_link    <- forearm_link
    #   joint_4 / revolute  / forearm_link  <- wrist_link
    #   joint_5 / revolute  / wrist_link    <- gripper_link
    #   joint_6 / prismatic / gripper_link  <- finger_r
    #   joint_7 / prismatic / gripper_link  <- finger_l
    joint_targets = [[  0.,
                        0.,
                        0.,
                        0.,
                        0.,
                      - 0.07,
                        0.07], \
                     [-45.*deg_to_rad,
                      -15.*deg_to_rad,
                       20.*deg_to_rad,
                       15.*deg_to_rad,
                      -75.*deg_to_rad,
                      - 0.03,
                        0.03], \
                     [ 30.*deg_to_rad,
                       60.*deg_to_rad,
                      -65.*deg_to_rad,
                       45.*deg_to_rad,
                        0.*deg_to_rad,
                      - 0.05,
                        0.05]]

    # Instantiate controller
    controller = ArmController()

    # Iterate through target joint positions
    for target in joint_targets:

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

    # Post simulation cleanup -- save results to a pickle, plot time histories, etc #####
    # Fill this out here (optional) or in your own script 
    # If you use a separate script, don't forget to include it in the deliverables
    # ...

    time_history = controller.history['timestamp']
    joint_feedback_history = controller.history['joint_feedback']
    joint_target_history = controller.history['joint_target']
    ctrl_command_history = controller.history['ctrl_commands']

    joint_feedback_history = np.array(joint_feedback_history)
    joint_target_history = np.array(joint_target_history)
    ctrl_command_history = np.array(ctrl_command_history)

    # plt.plot(time_history, test_error_list, '-b', label='test_error')
    plt.figure()
    plt.plot(time_history, joint_feedback_history[:, 0], 'r', linewidth=2, label='actual')
    plt.plot(time_history, joint_target_history[:, 0], 'b', linewidth=2, label='target')
    plt.xlabel('Timestamp (s)')
    plt.ylabel('Joint1 Angle (rad)')
    plt.legend()   
    plt.grid()


    plt.figure()
    plt.plot(time_history, joint_feedback_history[:, 1], 'r', linewidth=2, label='actual')
    plt.plot(time_history, joint_target_history[:, 1], 'b', linewidth=2, label='target')
    plt.xlabel('Timestamp (s)')
    plt.ylabel('Joint2 Angle (rad)')
    plt.legend()   
    plt.grid()
    
    plt.figure()
    plt.plot(time_history, joint_feedback_history[:, 2], 'r', linewidth=2, label='actual')
    plt.plot(time_history, joint_target_history[:, 2], 'b', linewidth=2, label='target')
    plt.xlabel('Timestamp (s)')
    plt.ylabel('Joint3 Angle (rad)')
    plt.legend()   
    plt.grid()

    plt.figure()
    plt.plot(time_history, joint_feedback_history[:, 3], 'r', linewidth=2, label='actual')
    plt.plot(time_history, joint_target_history[:, 3], 'b', linewidth=2, label='target')
    plt.xlabel('Timestamp (s)')
    plt.ylabel('Joint4 Angle (rad)')
    plt.legend()   
    plt.grid()

    plt.figure()
    plt.plot(time_history, joint_feedback_history[:, 4], 'r', linewidth=2, label='actual')
    plt.plot(time_history, joint_target_history[:, 4], 'b', linewidth=2, label='target')
    plt.xlabel('Timestamp (s)')
    plt.ylabel('Joint5 Angle (rad)')
    plt.legend()   
    plt.grid()

    plt.figure()
    plt.plot(time_history, joint_feedback_history[:, 5], 'r', linewidth=2, label='actual')
    plt.plot(time_history, joint_target_history[:, 5], 'b', linewidth=2, label='target')
    # plt.plot(time_history, ctrl_command_history[:, 5], 'g', linewidth=2, label='ctrl_command')
    plt.xlabel('Timestamp (s)')
    plt.ylabel('Joint6 Angle (m)')
    plt.legend()   
    plt.grid()

    plt.figure()
    plt.plot(time_history, joint_feedback_history[:, 6], 'r', linewidth=2, label='actual')
    plt.plot(time_history, joint_target_history[:, 6], 'b', linewidth=2, label='target')
    # plt.plot(time_history, ctrl_command_history[:, 6], 'g', linewidth=2, label='ctrl_command')
    plt.xlabel('Timestamp (s)')
    plt.ylabel('Joint7 Angle (m)')
    plt.legend()   
    plt.grid()

    plt.show()
    #####################################################################################


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    args = parser.parse_args()
    main(args)
