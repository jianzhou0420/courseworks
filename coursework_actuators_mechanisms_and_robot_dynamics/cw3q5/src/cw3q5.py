#!/usr/bin/python3.8
# -*- coding: utf-8 -*-
import rosbag
import rospkg
import numpy as np
import rospy
import rosbag
import rospkg
import matplotlib.pyplot as plt
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from cw3q2.iiwa14DynKDL import Iiwa14DynamicKDL
from cw3q2.iiwa14DynStudent import Iiwa14DynamicRef


class cw3q5(object):
    def __init__(self):
        self.cw3q2 = Iiwa14DynamicRef()
        self.acc_record = [[] for i in range(7)]
        self.start_time = 0
        self.is_started = False
        self.time_record = []

    def q5a(self):
        """
        Load bag file
        """
        # Get path
        handle = rospkg.RosPack()
        path = handle.get_path(name='cw3q5')
        bag = rosbag.Bag(path+'/bag/cw3q5.bag')
        for topic, msg, t in bag.read_messages(topics='/iiwa/EffortJointInterface_trajectory_controller/command'):
            traj = msg
        # Close bag

        bag.close()
        return traj

    def q5de(self, j_s):
        plt.clf()
        # Whether is the first time
        if self.is_started == False:
            self.is_started = True
            time1 = rospy.Time.now()
            self.start_time = time1.secs+time1.nsecs*1e-9

        # Calculate accelaration
        acc = self.calculate_acc(j_s)
        for i in range(7):
            self.acc_record[i].append(acc[i])

        # Calculate time
        time1 = rospy.Time.now()
        self.time_record.append(time1.secs+time1.nsecs*1e-9-self.start_time)

        # Plot
        plt.title('Joint Accelerations')
        for i in range(7):
            plt.subplot(7, 1, i+1)
            plt.plot(self.time_record, self.acc_record[i])

        plt.xlabel('Time')
        plt.ylabel('Acceleration')
        plt.draw
        plt.pause(1e-5)

        if self.time_record[-1] >= 50:
            rospy.on_shutdown(self.shutdown)
            rospy.signal_shutdown(
                'The node has been shutdown. Please refere to the acc.png for the final results')

    def calculate_acc(self, j_s):
        q = np.array(j_s.position)
        q_dot = np.array(j_s.velocity)
        tau = np.array(j_s.effort)

        B = Iiwa14DynamicKDL.get_B(Iiwa14DynamicKDL(), q.tolist())
        C_q_dot = Iiwa14DynamicKDL.get_C_times_qdot(
            Iiwa14DynamicKDL(), q.tolist(), q_dot.tolist())
        G = Iiwa14DynamicKDL.get_G(Iiwa14DynamicKDL(), q.tolist())
        acc = np.linalg.inv(B)@(tau-C_q_dot-G)
        acc = np.squeeze(np.array(acc))
        return acc

    def shutdown(self):
        plt.savefig('acc.png')
        print("the node is about to close")


if __name__ == '__main__':
    # 0. Initialise node
    rospy.init_node('cw3')
    cw3q5 = cw3q5()

    # 1. Create a publisher and publish to the robot
    publisher = rospy.Publisher(
        '/iiwa/EffortJointInterface_trajectory_controller/command', JointTrajectory, queue_size=5)
    traj = cw3q5.q5a()
    rospy.sleep(1)  # sleep to wait for the initialization of robot
    publisher.publish(traj)

    # 2. Create a subscriber and define callback function
    state_subscriber = rospy.Subscriber(
        '/iiwa/joint_states', JointState, cw3q5.q5de)

    # Finally: Other supportive functions
    plt.show()
    rospy.spin()
