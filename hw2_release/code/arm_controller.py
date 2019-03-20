import vrep_utils as vu

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