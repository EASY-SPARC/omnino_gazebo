#!/usr/bin/env python2

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from MPC import MPC

N = 10
N_c = 5
Ts = 0.1
X = np.array([0., 0.])
orientation = 0
V = np.array([0., 0.])
V_min = -5
V_max = 5

goal = np.array([float(sys.argv[1]), float(sys.argv[2])])
robot = sys.argv[3]

def cb_odom(msg):
    X[0] = msg.pose.pose.position.x
    X[1] = msg.pose.pose.position.y
    orientation = np.arctan2(2 * float(msg.pose.pose.orientation.w) * float(msg.pose.pose.orientation.z), 1 - 2 * float(msg.pose.pose.orientation.z)**2)

rospy.init_node('mpc_controller')

# Subscribing on model_states instead of robot/odom, to avoid unnecessary noise
rospy.Subscriber('/robot_'+robot+'/odom', Odometry, cb_odom)

# Velocity publishers
pub = rospy.Publisher('/robot_' + robot + '/cmd_vel', Twist, queue_size=10)

# Setpoint Publishers
pub_setpoint_pos = rospy.Publisher('/setpoint_pos', Vector3, queue_size=10)
pub_setpoint_vel = rospy.Publisher('/setpoint_vel', Vector3, queue_size=10)

setpoint_pos = Vector3()
setpoint_vel = Vector3()

# Initializing Controllers
controller = MPC(X, V_min, V_max, N, N_c, Ts)

# Global path planning
initial = np.copy(X)
t0 = 5.0
growth = 1
logistic = lambda t: 1/(1 + np.exp(- growth * (t - t0)))
d_logistic = lambda t: growth * logistic(t) * (1 - logistic(t))
P_des = lambda t: goal * logistic(t) + initial * (1 - logistic(t))
V_des = lambda t: goal * d_logistic(t) - initial * d_logistic(t)

t = 0

vel = Twist()

while not rospy.is_shutdown():

    # Updating setpoint trajectory
    setpoint = np.ravel([np.append(P_des(t + k * Ts), V_des(t + k * Ts)) for k in range(0, N + 1)])

    # Updating initial conditions
    controller.x_0 = np.array([X[0], X[1], V[0], V[1]])

    # Computing optimal input values
    [velocity, _] = controller.getNewVelocity(setpoint)

    [setpoint_pos.x, setpoint_pos.y] = P_des(t)

    [setpoint_vel.x, setpoint_vel.y] = V_des(t)

    #acc = accelerationTransform(acceleration, vel.linear.x, vel.angular.z, orientation)

    #vel.linear.x = vel.linear.x + acceleration[0] * Ts
    #vel.linear.y = vel.linear.y + acceleration[1] * Ts
    vel.linear.x = velocity[0]
    vel.linear.y = velocity[1]

    pub.publish(vel)

    pub_setpoint_pos.publish(setpoint_pos)
    pub_setpoint_vel.publish(setpoint_vel)
    rospy.sleep(Ts)

    t += Ts
