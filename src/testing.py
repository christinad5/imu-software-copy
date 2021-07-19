#!/usr/bin/env python3

import numpy as np
import math

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as R


def vec_to_quaternion(vec):
	quat = np.concatenate( (np.array([[0.0]]), vec), axis=0 )
	return quat


def get_matrix_vector_product(u):
	u1 = u[0][0]
	u2 = u[1][0]
	u3 = u[2][0]

	return np.array([[ 0.0, -u3, u2 ],
					[ u3, 0.0, -u1 ],
					[ -u2, u1, 0.0]])

def quat_left_multipl(p):
	p0 = p[0]
	pv = p[1:]
	p_L = np.block([
					[ p0, -pv.transpose()],
					[pv, p0*np.eye(3) + get_matrix_vector_product(pv)]
					])
	return p_L


def quat_right_multipl(q):
	q0 = q[0]
	qv = q[1:]
	q_R = np.block([
					[ q0, -qv.transpose()],
					[qv, q0*np.eye(3) - get_matrix_vector_product(qv)]
					])
	return q_R



def initial_estimation(y_acc, y_mag):
	'takes the initial acc and mag measurement as a np array col from vectornav'
	'return q estimate nb'


	g_hat_n = np.array([[0.0], [0.0], [1.0]])
	m_hat_n = np.array([[1.0], [0.0], [0.0]])
	g_hat_b = y_acc/np.linalg.norm(y_acc)

	y_mag_normalized = y_mag/np.linalg.norm(y_mag)

	m_hat_b = np.cross(g_hat_b, np.cross(y_mag_normalized, g_hat_b, axis=0), axis=0)

	g_hat_bar_n = vec_to_quaternion(g_hat_n)
	m_hat_bar_n = vec_to_quaternion(m_hat_n)
	g_hat_bar_b = vec_to_quaternion(g_hat_b)
	m_hat_bar_b = vec_to_quaternion(m_hat_b)

	A = -quat_left_multipl(g_hat_bar_n)@quat_right_multipl(g_hat_bar_b) - quat_left_multipl(m_hat_bar_n)@quat_right_multipl(m_hat_bar_b)
	w, v = np.linalg.eig(A)
	q_nb_1 = v[np.argmax(w), :] # eigenvector coresponding to the largest eigenvalue of A
	return q_nb_1

def initial_estimation_method_2(y_acc, y_mag):
	'takes the initial acc and mag measurement as a np array col from vectornav'
	'return q estimate nb'
	
	g_hat_n = np.array([[0.0], [0.0], [1.0]])
	m_hat_n = np.array([[1.0], [0.0], [0.0]])
	g_hat_b = y_acc/np.linalg.norm(y_acc)

	y_mag_normalized = y_mag/np.linalg.norm(y_mag)
	m_hat_b = np.cross(g_hat_b, np.cross(y_mag_normalized, g_hat_b, axis=0), axis=0)
	m_hat_b = m_hat_b/np.linalg.norm(m_hat_b)

	axis_hat_n = np.cross(g_hat_n, m_hat_n, axis=0)
	axis_hat_n = axis_hat_n/np.linalg.norm(axis_hat_n)
	axis_hat_b = np.cross(g_hat_b, m_hat_b, axis=0)
	axis_hat_b = axis_hat_b/np.linalg.norm(axis_hat_b)


	n_frame = np.block([ 
					[m_hat_n.transpose()],
					[axis_hat_n.transpose()],
					[g_hat_n.transpose()]
					])
	b_frame = np.block([ 
					[m_hat_b.transpose()],
					[axis_hat_b.transpose()],
					[g_hat_b.transpose()]
					])	

	rot = b_frame@n_frame.transpose()

	r = R.from_matrix(rot)

	return r.as_quat()


vector_imu_acc_xyz = np.array([[0.00001], [0.00001], [0.00001]])
vector_imu_mag_xyz = np.array([[1], [0.00001], [0.00001]])
quat_real = np.array([0.00001, 0.00001, 0.00001, 0.00001])

def callback_sensor_imu(data):
	global vector_imu_acc_xyz
	vector_imu_acc_xyz[0,0] = data.linear_acceleration.x 
	vector_imu_acc_xyz[1,0] = data.linear_acceleration.y
	vector_imu_acc_xyz[2,0] = data.linear_acceleration.z 
	# print(vector_imu_acc_xyz)


def callback_magnetic_field(data):
	global vector_imu_mag_xyz
	vector_imu_mag_xyz[0,0] = data.magnetic_field.x 
	vector_imu_mag_xyz[1,0] = data.magnetic_field.y
	vector_imu_mag_xyz[2,0] = data.magnetic_field.z 
	# print(vector_imu_mag_xyz)


def callback_orientation(data):
    global quat_real
    quat_real = np.array([data.orientation.x, data.orientation.y, 
    data.orientation.z, data.orientation.w])
	#end


def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/sensor/Imu", Imu, callback_orientation)
	rospy.Subscriber("/sensor/Imu", Imu, callback_sensor_imu)
	rospy.Subscriber("/sensor/Magnetometer", MagneticField, callback_magnetic_field)
    
	pub_quaternion_transform = rospy.Publisher('/initial_quat', Quaternion, queue_size=10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		q1 = initial_estimation(vector_imu_acc_xyz, vector_imu_mag_xyz)
		q2 = initial_estimation_method_2(vector_imu_acc_xyz, vector_imu_mag_xyz)
		
		quat1 = Quaternion()
		quat2 = Quaternion()
		quatreal = Quaternion()
		
		quat1.x = q1[1]
		quat1.y = q1[2]
		quat1.z = q1[3]
		quat1.w = q1[0]
		
		quat2.x = q2[0]
		quat2.y = q2[1]
		quat2.z = q2[2]
		quat2.w = q2[3]
		
		quatreal.x = quat_real[0]
		quatreal.y = quat_real[1]
		quatreal.z = quat_real[2]
		quatreal.w = quat_real[3]
		
		#pub_quaternion_transform.publish(quat1)
		pub_quaternion_transform.publish(quat2)
		#pub_quaternion_transform.publish(quatreal)
	rospy.spin()


if __name__ == '__main__':
	listener()