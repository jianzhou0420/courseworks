#!/usr/bin/env python3

import numpy as np
from cw2q4.youbotKineBase import YoubotKinematicBase


class YoubotKinematicStudent(YoubotKinematicBase):
    def __init__(self):
        super(YoubotKinematicStudent, self).__init__(tf_suffix='student')

        # Set the offset for theta --> This was updated on 23/11/2022. Feel free to use your own code.
        youbot_joint_offsets = [170.0 * np.pi / 180.0,
                                -65.0 * np.pi / 180.0,
                                146 * np.pi / 180,
                                -102.5 * np.pi / 180,
                                -167.5 * np.pi / 180]

        # Apply joint offsets to dh parameters
        self.dh_params['theta'] = [theta + offset for theta, offset in
                                   zip(self.dh_params['theta'], youbot_joint_offsets)]

        # Joint reading polarity signs
        self.youbot_joint_readings_polarity = [-1, 1, 1, 1, 1]

    def forward_kinematics(self, joints_readings, up_to_joint=5):
        """This function solve forward kinematics by multiplying frame transformation up until a specified
        frame number. The frame transformation used in the computation are derived from dh parameters and
        joint_readings.
        Args:
            joints_readings (list): the state of the robot joints. In a youbot those are revolute
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematics.
                Defaults to 5.
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix describing the pose of frame_{up_to_joint}
                w.r.t the base of the robot.
        """
        assert isinstance(self.dh_params, dict)
        assert isinstance(
            joints_readings, list), "joint readings of type " + str(type(joints_readings))
        assert isinstance(up_to_joint, int)
        assert up_to_joint >= 0
        assert up_to_joint <= len(self.dh_params['a'])

        T = np.identity(4)

        # --> This was updated updated on 23/11/2022. Feel free to use your own code.
        joints_readings = [sign * angle for sign,
                           angle in zip(self.youbot_joint_readings_polarity, joints_readings)]

        for i in range(up_to_joint):
            A = self.standard_dh(self.dh_params['a'][i],
                                 self.dh_params['alpha'][i],
                                 self.dh_params['d'][i],
                                 self.dh_params['theta'][i] + joints_readings[i])
            T = T.dot(A)

        assert isinstance(T, np.ndarray), "Output wasn't of type ndarray"
        assert T.shape == (4, 4), "Output had wrong dimensions"
        return T

    def get_jacobian(self, joint):
        """Given the joint values of the robot, compute the Jacobian matrix. Coursework 2 Question 4a.
        Reference - Lecture 5 slide 24.

        Args:
            joint (list): the state of the robot joints. In a youbot those are revolute

        Returns:
            Jacobian (numpy.ndarray): NumPy matrix of size 6x5 which is the Jacobian matrix.
        """
        assert isinstance(joint, list)
        assert len(joint) == 5

        # Your code starts here ----------------------------

        jacobian = np.zeros((6, 5))

        # Obtain End effector
        T05 = self.forward_kinematics(joint, up_to_joint=5)
        P0e = T05[0:3, -1]

        # For the first joint
        P0 = np.array([0, 0, 0]).reshape(3, 1)
        Z0 = np.array([0, 0, -1]).reshape(3, 1)
        jacobian[0:3, 0] = np.cross(Z0, (P0e - P0))
        jacobian[3:6, 0] = Z0

        # For the i-th joint
        for i in range(1, 5):
            T0i = self.forward_kinematics(joint, up_to_joint=i)
            Pi = T0i[0:3, -1]
            Zi = T0i[0:3, -2]
            jacobian[0:3, i] = np.cross(Zi, (P0e - Pi))
            jacobian[3:6, i] = Zi

            # For your solution to match the KDL Jacobian, z0 needs to be set [0, 0, -1] instead of [0, 0, 1], since that is how its defined in the URDF.
        # Both are correct.

        # Your code ends here ------------------------------

        assert jacobian.shape == (6, 5)
        return jacobian

    def check_singularity(self, joint):
        """Check for singularity condition given robot joints. Coursework 2 Question 4c.
        Reference Lecture 5 slide 30.

        Args:
            joint (list): the state of the robot joints. In a youbot those are revolute

        Returns:
            singularity (bool): True if in singularity and False if not in singularity.

        """
        assert isinstance(joint, list)
        assert len(joint) == 5

        # Your code starts here ----------------------------
        threshold = 1e-6
        singularity = False
        jacobian = self.get_jacobian(joint)

        if np.linalg.det(jacobian) < threshold:
            singularity = True
        # Your code ends here ------------------------------

        assert isinstance(singularity, bool)
        return singularity


# if __name__ == '__main__':
#     # try:
#     #     youbot_planner = YoubotTrajectoryPlanning()
#     #     youbot_planner.run()
#     #     rospy.spin()
#     # except rospy.ROSInterruptException:
#     #     pass

#     a = YoubotKinematicStudent()
#     a.get_jacobian()
