#!/usr/bin/env python2

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry, Path
from MPC import MPC

N = 10
N_c = 10
Ts = 0.1
t = 0
X = np.array([0., 0., 0.])
V = np.array([0., 0., 0.])
V_min = -5
V_max = 5
W_min = -0.3
W_max = 0.3
nx = 6                      #size of a state

goal = np.zeros(3)
robot = sys.argv[1]
path = Path()
setpoint = np.zeros((N+1)*nx)

DEBUG = False

def cb_odom(msg):
    X[0] = msg.pose.pose.position.x
    X[1] = msg.pose.pose.position.y
    X[2] = np.arctan2(2 * float(msg.pose.pose.orientation.w) * float(msg.pose.pose.orientation.z), 1 - 2 * float(msg.pose.pose.orientation.z)**2)

def cb_goal(msg):
	global V_des

	goal[0] = msg.goal.target_pose.pose.position.x
	goal[1] = msg.goal.target_pose.pose.position.y
	#msg.goal.target_pose.pose.position.z
	#msg.goal.target_pose.pose.orientation.x
	#msg.goal.target_pose.pose.orientation.y
	#msg.goal.target_pose.pose.orientation.z
	#msg.goal.target_pose.pose.orientation.w

	# Trajectory planning
	initial = np.copy(X)
	t0 = 5.0
	growth = 1
	logistic = lambda t: 1/(1 + np.exp(- growth * (t - t0)))
	d_logistic = lambda t: growth * logistic(t) * (1 - logistic(t))
	V_des = lambda t: goal * d_logistic(t) - initial * d_logistic(t)
	t = 0

	data = rospy.wait_for_message('/move_base/NavfnROS/plan', Path)
	cb_path(data)

def cb_path(msg):
    global path
    sub_sampling = 5

    for k in range(0,len(msg.poses),sub_sampling):
        path.poses.append(msg.poses[k])
    path.poses[-1] = msg.poses[-1]
 
    #setpoint = np.ravel([np.append(np.array([msg.poses[sub_sampling*k].pose.position.x, msg.poses[sub_sampling*k].pose.position.y, 0.], dtype=float), V_des(t + k * Ts)) for k in range(0, N+1)])

def velocity_transform(velocity, theta):
    robot_velocity = np.array([0., 0.])
    robot_velocity[0] = velocity[0]*np.cos(-theta) - velocity[1]*np.sin(-theta)
    robot_velocity[1] = velocity[0]*np.sin(-theta) + velocity[1]*np.cos(-theta)
    
    return robot_velocity



rospy.init_node('mpc_controller')

# Waiting gazebo first goal message
data = rospy.wait_for_message('/move_base/goal', MoveBaseActionGoal)
cb_goal(data)

# Subscribing on model_states instead of robot/odom, to avoid unnecessary noise
rospy.Subscriber('/robot_'+robot+'/odom', Odometry, cb_odom)

# Subscribing to goal
rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, cb_goal)

# Subscribing to full path
#rospy.Subscriber('/move_base/NavfnROS/plan', Path, cb_path)

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
    setpoint = np.ravel([np.append(np.array([path.poses[k].pose.position.x, path.poses[k].pose.position.y, 0.], dtype=float), V_des(t + k * Ts)) for k in range(0, N+1)])
    print(path.poses.pop(0), ">>" , path.poses[-1])
    #for k in range(0,N):
    	#setpoint[k*nx:(k+1)*nx] = setpoint[(k+1)*nx:(k+2)*nx]

    # Updating initial conditions
    controller.x_0 = np.array([X[0], X[1], X[2], V[0], V[1], V[2]])

    # Computing optimal input values
    [velocity, _, angular] = controller.getNewVelocity(setpoint)

    # Remember, sepoint_pos.z reffers to orientation
    [setpoint_pos.x, setpoint_pos.y, setpoint_pos.z] = setpoint[0:3]
    orientation.data = X[2]

    [setpoint_vel.x, setpoint_vel.y, setpoint_vel.z] = V_des(t)

    velocity = velocity_transform(velocity, X[2])

    vel.linear.x = velocity[0]
    vel.linear.y = velocity[1]
    vel.angular.z = angular

    pub.publish(vel)

    pub_setpoint_pos.publish(setpoint_pos)
    pub_setpoint_vel.publish(setpoint_vel)
    pub_orientation.publish(orientation)
    rospy.sleep(Ts)

    t += Ts
