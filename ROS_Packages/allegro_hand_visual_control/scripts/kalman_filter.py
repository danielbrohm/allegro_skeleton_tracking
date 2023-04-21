#!/usr/bin/python3

# This is a ROS-node (kalman_filter) which takes joint positions from the state_publisher_hand node, puts them into a Kalman filter 
# and publishes the filtered joint positions
# The code for the Kalman filter was created by using the template from https://thekalmanfilter.com/kalman-filter-python-example/ 
# and adapted to work for the 16 joint positions of the allegro hand

import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
#from ros_openpose.msg import Frame

class Kalman_Filter:
    def __init__(self):
        self.updated_joint_positions = Float64MultiArray()
        self.joint_positions = np.array([0.0, 0.1, 0.4, 0.1, 0.0, 0.1, 0.4, 0.1, 0.0, 0.1, 0.4, 0.1, np.pi/4, np.pi/4, np.pi/4, -np.pi/4])
        self.updated_joint_positions.data = np.array([0.0, 0.1, 0.4, 0.1, 0.0, 0.1, 0.4, 0.1, 0.0, 0.1, 0.4, 0.1, np.pi/4, np.pi/4, np.pi/4, -np.pi/4])
        self.dt = 1/12  # dt = 1/ros-rate
        # x (2x1) and P (2x2) have to be created for every joint
        # The filter function takes only a part of the array according to the joint-number
        self.filter_x = np.array([[0.0, 0.1, 0.4, 0.1, 0.0, 0.1, 0.4, 0.1, 0.0, 0.1, 0.4, 0.1, -1.8, 0.1, 0.1, 0.2], [0]*16]) 
        # x is the two element state vector for position and velocity.
        self.filter_P = np.array([[5, 0]*16,[0, 5]*16]) # P is the 2×2 state covariance matrix representing the uncertainty in x.

        self.filter_A = np.array([[1, self.dt], [0, 1]])    # A is the state transition matrix for a system model that assumes constant linear motion.
        self.filter_H = np.array([[1, 0]])  # H is the state to measurement transition matrix.
        self.filter_HT = np.array([[1], [0]])   # HT is the H matrix transposed
        self.filter_R = np.array([10.0]*16)  # R is the input measurement variance.
        self.filter_Q = np.array([[1, 0], [0, 1]])   # Q is the 2×2 system noise covariance matrix. Q accounts for inaccuracy in the system model.
        self.sub = rospy.Subscriber('joint_prekalman_states', JointState, self.callback)
        self.pub = rospy.Publisher("joint_postkalman_states", Float64MultiArray, queue_size = 16)

        #self.sub = rospy.Subscriber('/frame', Frame, self.ScoreCallback)   #could be used to change the input measurement variance according to 
                                                                                    #the score (accuracy) received by ros_openpose

        self.rate = rospy.Rate(12)  # same ros-rate as in state_publisher.cpp

    def callback(self,msg):
        self.joint_positions = msg.position

    def ScoreCallback(self,msg):
        added_scores = 0.0
        if len(msg.persons)>0:
            for i in range(16):
                """
                if msg.persons[0].rightHandParts[i+1].score > 0:
                    self.filter_R[i] = 2.0/msg.persons[0].rightHandParts[i+1].score
                else:
                    self.filter_R[i] = 100000
                print(self.filter_R[i])
                """
                added_scores = added_scores + msg.persons[0].rightHandParts[i+1].score
        added_scores = added_scores/16.0
        if added_scores > 0.0:
            variance = 10/added_scores
        else:
            variance = 100.0
        print(variance)
        self.filter_R = np.array([variance]*16)

    def run(self):
        while not rospy.is_shutdown():
            for i in range(0,16):
                self.updated_joint_positions.data[i] = self.filter_func(self.joint_positions[i], i)

            self.pub.publish(self.updated_joint_positions)
            self.rate.sleep()

    def filter_func(self, y, joint_number):

        # Predict State Forward
        x_p = self.filter_A.dot(self.filter_x[:, joint_number])
        x_p = np.array([[x_p[0]], [x_p[1]]])
        # Predict Covariance Forward
        P_p = self.filter_A.dot(self.filter_P[:, joint_number*2:(joint_number+1)*2]).dot(self.filter_A.T) + self.filter_Q
        # Compute Kalman Gain
        S = self.filter_H.dot(P_p).dot(self.filter_HT) + self.filter_R[joint_number]
        K = P_p.dot(self.filter_HT)*(1/S)

        # Estimate State
        residual = y - self.filter_H.dot(x_p)
        result_x = np.add(x_p, K*residual)
        self.filter_x[0, joint_number] = result_x[0]
        self.filter_x[1, joint_number] = result_x[1]

        # Estimate Covariance
        self.filter_P[:, joint_number*2:(joint_number+1)*2] = P_p - K.dot(self.filter_H).dot(P_p)

        return self.filter_x[0, joint_number]

if __name__ == '__main__':
    rospy.init_node('kalman_filter', anonymous=True)
    kalman_object = Kalman_Filter()
    kalman_object.run()
