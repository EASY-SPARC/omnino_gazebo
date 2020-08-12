#!/usr/bin/env python2

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry, Path
from MPC import MPC

N = 50
N_c = 50
Ts = 0.1
X = np.array([0., 0., 0.])
V = np.array([0., 0., 0.])
V_min = -5
V_max = 5
W_min = -0.3
W_max = 0.3
nx = 6                      #size of a state

goal = np.array([float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])])
robot = sys.argv[4]
setpoint = np.zeros((N+1)*nx)

DEBUG = False

def cb_odom(msg):
    X[0] = msg.pose.pose.position.x
    X[1] = msg.pose.pose.position.y
    X[2] = np.arctan2(2 * float(msg.pose.pose.orientation.w) * float(msg.pose.pose.orientation.z), 1 - 2 * float(msg.pose.pose.orientation.z)**2)

#k*(len(msg.poses)-1)/N working     50 + k*(len(msg.poses)-51)/N
def cb_path(msg):
    global setpoint
    setpoint = np.ravel([np.append(np.array([msg.poses[k*(len(msg.poses)-1)/N].pose.position.x*np.cos(X[2]) - msg.poses[k*(len(msg.poses)-1)/N].pose.position.y*np.sin(X[2]) + X[0], msg.poses[k*(len(msg.poses)-1)/N].pose.position.y*np.cos(X[2]) + msg.poses[k*(len(msg.poses)-1)/N].pose.position.x*np.sin(X[2]) + X[1], 0.], dtype=float), np.array([0., 0., 0.], dtype=float)) for k in range(0, N+1)])
    #print (msg.header.seq)
    #print("AQUI")

def velocity_transform(velocity, theta):
    robot_velocity = np.array([0., 0.])
    robot_velocity[0] = velocity[0]*np.cos(-theta) - velocity[1]*np.sin(-theta)
    robot_velocity[1] = velocity[0]*np.sin(-theta) + velocity[1]*np.cos(-theta)
    
    return robot_velocity

#if DEBUG:
# Global path planning
initial = np.copy(X)
t0 = 5.0
growth = 1
logistic = lambda t: 1/(1 + np.exp(- growth * (t - t0)))
d_logistic = lambda t: growth * logistic(t) * (1 - logistic(t))
P_des = lambda t: goal * logistic(t) + initial * (1 - logistic(t))
V_des = lambda t: goal * d_logistic(t) - initial * d_logistic(t)
t = 0

rospy.init_node('mpc_controller')

if not DEBUG:
	# Waiting gazebo first message
	data = rospy.wait_for_message('/move_base/TrajectoryPlannerROS/global_plan', Path)
	cb_path(data)

# Subscribing on model_states instead of robot/odom, to avoid unnecessary noise
rospy.Subscriber('/robot_'+robot+'/odom', Odometry, cb_odom)
if not DEBUG:
	rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, cb_path)

# Velocity publishers
pub = rospy.Publisher('/robot_' + robot + '/cmd_vel', Twist, queue_size=10)

# Setpoint Publishers
pub_setpoint_pos = rospy.Publisher('/setpoint_pos', Vector3, queue_size=10)
pub_setpoint_vel = rospy.Publisher('/setpoint_vel', Vector3, queue_size=10)
pub_orientation = rospy.Publisher('/orientation', Float64, queue_size=10)

setpoint_pos = Vector3()
setpoint_vel = Vector3()
orientation = Float64()

# Initializing Controllers
controller = MPC(X, V_min, V_max, W_min, W_max, N, N_c, Ts)

vel = Twist()

while not rospy.is_shutdown():
    
    # Updating setpoint trajectory
    if DEBUG:
    	setpoint = np.ravel([np.append(P_des(t + k * Ts), V_des(t + k * Ts)) for k in range(0, N + 1)])
    else:
        for k in range(0,N):
            setpoint[k*nx:(k+1)*nx] = setpoint[(k+1)*nx:(k+2)*nx]

    # Updating initial conditions
    controller.x_0 = np.array([X[0], X[1], X[2], V[0], V[1], V[2]])

    # Computing optimal input values
    [velocity, _, angular] = controller.getNewVelocity(setpoint)

    [setpoint_pos.x, setpoint_pos.y, setpoint_pos.z] = P_des(t)
    orientation.data = X[2]

    [setpoint_vel.x, setpoint_vel.y, setpoint_vel.z] = V_des(t)

    #acc = accelerationTransform(acceleration, vel.linear.x, vel.angular.z, orientation)
    velocity = velocity_transform(velocity, X[2])

    #vel.linear.x = vel.linear.x + acceleration[0] * Ts
    #vel.linear.y = vel.linear.y + acceleration[1] * Ts
    vel.linear.x = velocity[0]
    vel.linear.y = velocity[1]
    vel.angular.z = angular

    pub.publish(vel)

    pub_setpoint_pos.publish(setpoint_pos)
    pub_setpoint_vel.publish(setpoint_vel)
    pub_orientation.publish(orientation)
    rospy.sleep(Ts)

    t += Ts
