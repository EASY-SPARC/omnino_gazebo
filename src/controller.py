#!/usr/bin/env python2

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

goal = np.array([float(sys.argv[1]), float(sys.argv[2])])                   #goal are given global coordinates
robot = sys.argv[3]                                                         #robot is used as identification in topics and other functionalities

def cb_odom(msg):
    robot_coordinates[0] = msg.pose.pose.position.x
    robot_coordinates[1] = msg.pose.pose.position.y
    ######### Assuming fixed orientation as 0 #########
    robot_coordinates[2] = 0
    
#############################
###### VELOCITY CONTROL #####
#############################
def vel_controll(goal, robot_coordinates, K_p):
    error_x = goal[0] - robot_coordinates[0]                                #coordinate errors between robot location and goal
    error_y = goal[1] - robot_coordinates[1]
    theta = robot_coordinates[2]                                            #robot orientation
    robot_vel.linear.x = K_p*(error_x*np.cos(theta) - error_y*np.sin(theta))#velocity is given proportional to the error
    robot_vel.linear.y = K_p*(error_x*np.sin(theta) + error_y*np.cos(theta))#and rotated to robot's frame

#############################
###### WORLD VARIABLES ######
#############################
robot_coordinates = np.zeros(3)                                             #Robot's global location and orientation
robot_vel = Twist()                                                         #Velocity in robot's frame
K_p = 0.5                                                                   #Proportional gain used in velocity controll

#############################
###### PREPARING ROSS #######
#############################
rospy.init_node('controller')

rospy.Subscriber('/robot_'+robot+'/odom', Odometry, cb_odom)

pub = rospy.Publisher('/robot_' + robot + '/cmd_vel', Twist, queue_size=10)

while not rospy.is_shutdown():
    vel_controll(goal, robot_coordinates, K_p)
    pub.publish(robot_vel)

rospy.spin()

#data = rospy.wait_for_message('/robot_'+robot+'/odom', Odometry)

#print(data.pose.pose.position)

