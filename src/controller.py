#!/usr/bin/env python2

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

goal = np.array([float(sys.argv[1]), float(sys.argv[2])])
robot = sys.argv[3]

def cb_odom(msg):
    robot_coordinates[0] = msg.pose.pose.position.x
    robot_coordinates[1] = msg.pose.pose.position.y
    ######### Assuming unaltered orientation as pi/2 #########
    robot_coordinates[2] = -np.pi/2
    
def vel_controll(goal, robot_coordinates, K_p):
    error_x = goal[0] - robot_coordinates[0]
    error_y = goal[1] - robot_coordinates[1]
    theta = robot_coordinates[2]
    robot_vel.linear.x = K_p*(error_x*np.cos(theta) - error_y*np.sin(theta))
    robot_vel.linear.y = K_p*(error_x*np.sin(theta) + error_y*np.cos(theta))


robot_coordinates = np.zeros(3)
robot_vel = Twist()
K_p = 0.5

rospy.init_node('controller')

rospy.Subscriber('/robot_'+robot+'/odom', Odometry, cb_odom)

pub = rospy.Publisher('/robot_' + robot + '/cmd_vel', Twist, queue_size=10)

while not rospy.is_shutdown():
    vel_controll(goal, robot_coordinates, K_p)
    pub.publish(robot_vel)

rospy.spin()

#data = rospy.wait_for_message('/robot_'+robot+'/odom', Odometry)

#print(data.pose.pose.position)

