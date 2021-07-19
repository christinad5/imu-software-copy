#!/usr/bin/env python3

"""This code takes in orientation data from Vectornav and gives the corresponding rotation matrix"""

import numpy as np
import math

import rospy
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Quaternion

#rot_matrix = Float32MultiArray()
q = Quaternion()
quat_real = [0, 0, 0, 1]

def from_quat_to_matrix(q):
    q0 = q[3]
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    R = np.array([[2*np.square(q0)+2*np.square(q1)-1, 2*q1*q2-2*q0*q3, 2*q1*q3+2*q0*q2],
                [2*q1*q2+2*q0*q3, 2*np.square(q0)+2*np.square(q2)-1, 2*q2*q3-2*q0*q1],
                [2*q1*q3-2*q0*q2, 2*q2*q3+2*q0*q1, 2*np.square(q0)+2*np.square(q3)-1]])
    return R


def callback_orientation(data):
    global quat_real
    quat_real = np.array([data.orientation.x, data.orientation.y, 
    data.orientation.z, data.orientation.w])
    global rot_matrix
    rot_matrix = from_quat_to_matrix(quat_real)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/sensor/Imu", Imu, callback_orientation)
    
    pub_quaternion_transform = rospy.Publisher('/initial_quat', Quaternion, queue_size=10)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        q.x = quat_real[0]
        q.y = quat_real[1]
        q.z = quat_real[2]
        q.w = quat_real[3]

        rospy.loginfo(q)
        pub_quaternion_transform.publish(q)
    rospy.spin()


if __name__ == '__main__':
	listener()

#matrix = Float32MultiArray([3,3], rot_matrix)
#rospy.loginfo(matrix)