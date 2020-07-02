#!/usr/bin/env python2
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

r = 0.0175
d = 0.063

def cb(msg):
	v_b = np.array([float(msg.angular.z), float(msg.linear.y), -float(msg.linear.x)])
	u = Float64MultiArray()
	u.data = np.zeros(3)
	u.data[0] = (1/r) * (-d * v_b[0] + v_b[1])
	u.data[1] = (1/r) * (-d * v_b[0] - 0.5 * v_b[1] - np.sin(np.pi/3) * v_b[2])
	u.data[2] = (1/r) * (-d * v_b[0] - 0.5 * v_b[1] + np.sin(np.pi/3) * v_b[2])
	u.data = -u.data
	pub.publish(u)

rospy.init_node('inv_kine')

rospy.Subscriber('/robot_1/cmd_vel', Twist, cb)

pub = rospy.Publisher('/robot_1/wheels_velocity_controller/command', Float64MultiArray, queue_size=10)

rospy.spin()
