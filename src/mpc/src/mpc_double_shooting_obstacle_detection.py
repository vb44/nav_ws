#!/usr/bin/env python

# ROS imports
import rospy
from geometry_msgs.msg import Twist, Pose
from custom_msgs.msg import Cluster, Predict
# from cluster_msgs.msg import Cluster, Predict

# MPC solver imports
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import math

###############################################################################
# MPC settings
###############################################################################
N = 20                  # prediction horizon
ROB_DIAM = 0.5          # diameter of the robot [m]
STEP_HORIZON = 0.2      # was 1 for practical testing to allow for larger prediction horizon

# mpc solver options
opts = {
    'ipopt': {
        'max_iter': 2000,
        'print_level': 0,
        'acceptable_tol': 1e-8,
        'acceptable_obj_change_tol': 1e-6
    },
    'print_time': 0
}

###############################################################################
# navigation initial and final goal
###############################################################################
X_INIT = 0
Y_INIT = 0
THETA_INIT = 0
X_TARGET = 3.5
Y_TARGET = 3.5
THETA_TARGET = 0
FINAL_POSE_TOLERANCE = 1e-2

###############################################################################
# control input limits
###############################################################################
V_MAX = 0.2
V_MIN = -V_MAX          # minimum robot linear velocity
OMEGA_MAX = pi/6        # maximum robot angular velocity
OMEGA_MIN = -OMEGA_MAX  # minimum robot linear velocity

###############################################################################
# controller weights
###############################################################################
# state weights matrix (Q_X, Q_Y, Q_THETA)
Q_X = 10
Q_Y = 10
Q_THETA = 0.1
Q = ca.diagcat(Q_X, Q_Y, Q_THETA)

# controls weights matrix
R1 = 0.5
R2 = 0.5
R = ca.diagcat(R1, R2)

###############################################################################
# map bounds for each state
###############################################################################
X_LOWER = -20 # m
Y_LOWER = -20 
THETA_LOWER = -ca.inf
X_UPPER = 20
Y_UPPER = 20
THETA_UPPER = ca.inf

###############################################################################
# obstacle modelling
###############################################################################
OBS_DIAM = 0.2

###############################################################################
# helper functions
###############################################################################

# update the state for the MPC predictions
def shift_timestep(u):
    u0 = ca.horzcat(
        u[:, 1:],
        ca.reshape(u[:, -1], -1, 1)
    )
    return u0

# convert DM to array
def DM2Arr(dm):
    return np.array(dm.full())

# convert from Quaternion to Euler angles
# from automaticaddison.com
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

###############################################################################
# state symbolic variables
###############################################################################
x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')
states = ca.vertcat(
    x,
    y,
    theta
)
n_states = states.numel() # no. of states

# control symbolic variables
v = ca.SX.sym('v')
omega = ca.SX.sym('omega')
controls = ca.vertcat(
    v,
    omega
)
n_controls = controls.numel()

# matrix containing all states over all time steps +1 (each column is a state vector)
X = ca.SX.sym('X', n_states, N+1)

# matrix containing all control actions over all time steps (each column is a control vector)
U = ca.SX.sym('U', n_controls, N)

# column vector for storing initial state and target state
P = ca.SX.sym('P', n_states + n_states)

# Differential drive transfer function
J = ca.vertcat(
    ca.horzcat(cos(theta), 0),
    ca.horzcat(sin(theta), 0),
    ca.horzcat(0, 1)
)

# RHS
RHS = J @ controls

# maps controls from [v, omega].T to [vx, vy, omega].T
f = ca.Function('f', [states, controls], [RHS])

###############################################################################
# compute solution symbolically
###############################################################################
obj = 0  # cost function
g = X[:, 0] - P[:n_states]  # constraints in the equation

# calculate the next N predictions
# use forward recursion to fill our X - determine prediction
G_COUNTER = 1
for k in range(N):
    st = X[:, k] # state
    con = U[:, k] #  control input
    obj = obj + (st-P[3:6]).T @ Q @ (st-P[3:6]) + con.T @ R @ con
    st_next = X[:, k+1]
    f_value = f(st, con)
    st_next_euler = st + (STEP_HORIZON*f_value)
    g = ca.vertcat(g, st_next-st_next_euler)
    G_COUNTER = G_COUNTER + 1

OPT_variables = ca.vertcat(
    X.reshape((-1, 1)),   
    U.reshape((-1, 1))
)

# equality constraints
lbg = ca.DM.zeros((n_states*(N+1), 1))
ubg = ca.DM.zeros((n_states*(N+1), 1))

# state lower and upper bounds
lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))

lbx[0: n_states*(N+1): n_states] = X_LOWER          # X lower bound
lbx[1: n_states*(N+1): n_states] = Y_LOWER          # Y lower bound
lbx[2: n_states*(N+1): n_states] = THETA_LOWER      # theta lower bound

ubx[0: n_states*(N+1): n_states] = X_UPPER           # X upper bound
ubx[1: n_states*(N+1): n_states] = Y_UPPER           # Y upper bound
ubx[2: n_states*(N+1): n_states] = THETA_UPPER       # theta upper bound

lbx[n_states*(N+1): n_states*(N+1)+2*N: n_controls] = V_MIN         # v lower bound for all V
ubx[n_states*(N+1): n_states*(N+1)+2*N: n_controls] = V_MAX
lbx[n_states*(N+1)+1: n_states*(N+1)+2*N: n_controls] = OMEGA_MIN
ubx[n_states*(N+1)+1: n_states*(N+1)+2*N: n_controls] = OMEGA_MAX

###############################################################################
# MPC Initialisation
###############################################################################
state_init = ca.DM([X_INIT, Y_INIT, THETA_INIT]) # initial state
state_target = ca.DM([X_TARGET, Y_TARGET, THETA_TARGET]) # target state

u0 = ca.DM.zeros((n_controls, N))   # initial control
X0 = ca.repmat(state_init, 1, N+1)  # initial state full

def add_obstacle(obstacle_pos):
    global lbg, ubg, g
    g = g[0:(G_COUNTER)*n_states]
    NO_OF_OBSTACLES = 0
    if len(obstacle_pos) > 0:
        RUN_ONCE = True
        for k in (range(N+1)):
            for i in range(0, len(obstacle_pos), 2):
                obs_x = obstacle_pos[i]
                obs_y = obstacle_pos[i+1]
                if RUN_ONCE:
                    NO_OF_OBSTACLES = NO_OF_OBSTACLES + 1
                g = ca.vertcat(g, -1*np.sqrt(pow((X[0,k]-obs_x), 2) + pow((X[1,k]-obs_y),2)) + (ROB_DIAM/2 + OBS_DIAM/2))
            RUN_ONCE = False
        lbg = ca.DM.zeros((n_states*(N+1)+(N+1)*NO_OF_OBSTACLES, 1)) 
        ubg = ca.DM.zeros((n_states*(N+1)+(N+1)*NO_OF_OBSTACLES, 1))

        # inequality constraints
        lbg[n_states*(N+1): n_states*(N+1)+(N+1)*NO_OF_OBSTACLES] = -ca.inf
        ubg[n_states*(N+1): n_states*(N+1)+(N+1)*NO_OF_OBSTACLES] = 0
    else:
        lbg = ca.DM.zeros((n_states*(N+1), 1))
        ubg = ca.DM.zeros((n_states*(N+1), 1))
            
CLUST_CENTRES = []
def update_obstacles(data):
    global CLUST_CENTRES
    CLUST_CENTRES = list(data.cluster_centres)
    

###############################################################################
# MPC Main Loop
###############################################################################
def callback(data):
    global u0, X0, counter, g
    X_INIT = data.position.x
    Y_INIT = data.position.y
    roll, pitch, yaw = euler_from_quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
    THETA_INIT = yaw
    state_init = ca.DM([X_INIT, Y_INIT, THETA_INIT])
    rospy.loginfo("%lf, %lf, %lf", X_INIT, Y_INIT, THETA_INIT)

    ##########################################################################
    # MPC Solver
    ###########################################################################
    add_obstacle(CLUST_CENTRES)
    
    nlp_prob = {
    'f': obj,
    'x': OPT_variables,
    'g': g,
    'p': P
    }

    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

    args = {
    'lbg': lbg, 
    'ubg': ubg, 
    'lbx': lbx,
    'ubx': ubx
    }

    ###########################################################################
    # Apply control actions
    ###########################################################################
    if (ca.norm_2(state_init - state_target) > FINAL_POSE_TOLERANCE):
        args['p']  = ca.vertcat(
            state_init, # current state
            state_target # target state
        )

        # optimization variable current state
        args['x0'] = ca.vertcat(
            ca.reshape(X0, n_states*(N+1), 1),
            ca.reshape(u0, n_controls*N, 1)
        )
        
        sol = solver(
            x0=args['x0'],
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg'],
            p=args['p']
        )


        u = ca.reshape(sol['x'][n_states * (N + 1):], n_controls, N)
        X0 = ca.reshape(sol['x'][: n_states * (N+1)], n_states, N+1)
        u0 = shift_timestep(u)

        predict = (np.array(X0)).tolist()
        x_predict = predict[0]
        y_predict = predict[1]
        theta_predict = predict[2]
        control_inputs = u[0:2]
        v_desired = control_inputs[0];
        omega_desired = control_inputs[1];
    else:
        v_desired = 0
        omega_desired = 0    
   
    # publish prediction for plotting
    predict = Predict()
    predict.x_predict = x_predict
    predict.y_predict = y_predict
    predict.theta_predict = theta_predict
    pub_predict.publish(predict)

    # publish twist  message to robot
    twist = Twist()
    twist.linear.x = v_desired
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = omega_desired
    pub.publish(twist)
    # rate.sleep()

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
pub_predict = rospy.Publisher('predict', Predict, queue_size=10)

def ground_truth_listener():
    rospy.init_node('ground_truth_listener', anonymous=True)
    rospy.Subscriber("robot_pos", Pose, callback, queue_size=1)
    rospy.Subscriber("clusters", Cluster, update_obstacles)
    rospy.spin()

if __name__=='__main__':
    try:
        ground_truth_listener()
    except rospy.ROSInterruptException:
        pass