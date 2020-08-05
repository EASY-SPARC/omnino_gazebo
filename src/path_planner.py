#!/usr/bin/env python2

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry, Path


Ts = 0.1
X = np.array([0., 0., 0.])
N = 10

goal = np.array([float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])])
robot = sys.argv[4]

def cb_odom(msg):
    X[0] = msg.pose.pose.position.x
    X[1] = msg.pose.pose.position.y
    X[2] = np.arctan2(2 * float(msg.pose.pose.orientation.w) * float(msg.pose.pose.orientation.z), 1 - 2 * float(msg.pose.pose.orientation.z)**2)

rospy.init_node('path_planner')

# Subscribing on model_states instead of robot/odom, to avoid unnecessary noise
rospy.Subscriber('/robot_'+robot+'/odom', Odometry, cb_odom)

#Trajectory Publisher
pub = rospy.Publisher('/robot_' + robot + '/trajectory', Path, queue_size=10)

# Setpoint Publishers
pub_setpoint_pos = rospy.Publisher('/setpoint_pos', Vector3, queue_size=10)
pub_setpoint_vel = rospy.Publisher('/setpoint_vel', Vector3, queue_size=10)
pub_orientation = rospy.Publisher('/orientation', Float64, queue_size=10)

setpoint_pos = Vector3()
setpoint_vel = Vector3()
orientation = Float64()

# Global path planning
initial = np.copy(X)
t0 = 5.0
growth = 1
logistic = lambda t: 1/(1 + np.exp(- growth * (t - t0)))
d_logistic = lambda t: growth * logistic(t) * (1 - logistic(t))
P_des = lambda t: goal * logistic(t) + initial * (1 - logistic(t))
V_des = lambda t: goal * d_logistic(t) - initial * d_logistic(t)
t = 0

trajectory = Path()

while not rospy.is_shutdown():
    
    for k in range(0, N + 1):
        point = PoseStamped()
        point.pose.position = P_des(t + k * Ts)
        trajectory.poses.append(point.pose.position)
        
    pub.publish(trajectory)

    [setpoint_pos.x, setpoint_pos.y, setpoint_pos.z] = P_des(t)
    orientation.data = X[2]

    [setpoint_vel.x, setpoint_vel.y, setpoint_vel.z] = V_des(t)

    pub_setpoint_pos.publish(setpoint_pos)
    pub_setpoint_vel.publish(setpoint_vel)
    pub_orientation.publish(orientation)
    rospy.sleep(Ts)

    t += Ts
