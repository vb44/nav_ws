# !/usr/bin/env python

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ros imports
import rospy
from geometry_msgs.msg import Pose
from custom_msgs.msg import Cluster, Predict
from sensor_msgs.msg import LaserScan

# robot position index
POS_X = 0 # meters
POS_Y = 0 # meters
POS_THETA = 0 # radians

# goal and end poses
START = [0,0]           # plot and label start and end positions
TARGET = [3.5,3.5] 

# plot axes limits
X_LOWER = -1
Y_LOWER = -1
X_UPPER = 4
Y_UPPER = 4

# convert from quaternion to euler angles
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

class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.robot_pos, = plt.plot([], [], 'go', markersize=20, fillstyle='full')   # robot position
        self.predict, = plt.plot([], [], 'mo--', markersize=20, fillstyle='none')   # mpc prediction
        self.lidar_points, = plt.plot([], [], 'ro', markersize=1)                   # lidar_points
        self.clusters, = plt.plot([], [], 'bo', markersize=10, fillstyle='none')    # clusters
        self.target, = plt.plot([], [], 'bx', markersize=20)                        # start and end points  
        self.fig.suptitle('Robot Pose in the World Frame')                  
        self.ax.set_xlabel('x (m)')
        self.ax.set_ylabel('y (m)')
        self.x_pos, self.y_pos = [], []                                             # robot position data
        self.x_predict, self.y_predict = [], []                                     # mpc prediction data
        self.x_lidar_data, self.y_lidar_data = [], []
        self.x_clus_data, self.y_clus_data = [], []

    def plot_init(self):
        self.ax.set_xlim(X_LOWER, X_UPPER)
        self.ax.set_ylim(Y_LOWER, Y_UPPER)
        plt.grid(color='k', linestyle='-', linewidth=0.2)                           # set grid
        plt.gca().set_aspect('equal', adjustable='box')                             # axis equal                                 
        self.target.set_data([START[0], TARGET[0]], [START[1], TARGET[1]])
        
    def update_plot(self, frame):
        rospy.loginfo("%lf, %lf", len(self.x_pos), len(self.y_pos))
        self.robot_pos.set_data(POS_X, POS_Y)
        self.predict.set_data(self.x_predict, self.y_predict)
        self.lidar_points.set_data(self.x_lidar_data, self.y_lidar_data)
        self.clusters.set_data(self.x_clus_data, self.y_clus_data)

    def update_data(self, data):
        global POS_X, POS_Y, POS_THETA
        roll, pitch, yaw  = euler_from_quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        POS_X = data.position.x
        POS_Y = data.position.y
        self.x_pos = []
        self.y_pos = []
        self.x_pos.append(POS_X)
        self.y_pos.append(POS_Y)
        POS_THETA = yaw

    def update_predict(self, data):
        self.x_predict = data.x_predict[0:len(data.x_predict):4]
        self.y_predict = data.y_predict[0:len(data.x_predict):4]
        rospy.loginfo("%lf, %lf", len(self.x_predict), len(self.y_predict))


    def scan_callback(self, msg):
        rot = np.array([[np.cos(POS_THETA), -np.sin(POS_THETA)], [np.sin(POS_THETA), np.cos(POS_THETA)]])
        self.x_lidar_data = []
        self.y_lidar_data = []
        scan = msg.ranges
        scan_cartesian = []
        theta = np.linspace(0, 2*np.pi, num=360)
        for i in range(0, len(scan), 1):
            if (scan[i] != float("inf")):
                cart_point = np.array([(scan[i] * np.cos(theta[i])), (scan[i] * np.sin(theta[i]))])
                point = rot @ cart_point.T
                self.x_lidar_data.append(point[0] + POS_X)
                self.y_lidar_data.append(point[1] + POS_Y)

    def cluster_callback(self, msg):
        self.x_clus_data = []
        self.y_clus_data = []
        points = msg.cluster_centres
        for i in range(0, len(points), 2):
            self.x_clus_data.append(points[i])
            self.y_clus_data.append(points[i+1])


rospy.init_node('mpc_plotter')
vis = Visualiser()

# setup subscribers
rospy.Subscriber("robot_pos", Pose, vis.update_data, queue_size=1)          # robot state
rospy.Subscriber("predict", Predict, vis.update_predict, queue_size=1)      # mpc prediction states
rospy.Subscriber('scan', LaserScan, vis.scan_callback, queue_size=1)        # LiDAR scan data
rospy.Subscriber('clusters', Cluster, vis.cluster_callback, queue_size=1)   # detected obstacle position

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True)