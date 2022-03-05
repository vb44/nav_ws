#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from custom_msgs.msg import Cluster
from geometry_msgs.msg import Pose
import math
import numpy as np

# robot position
POS_X = 0       # meters
POS_Y = 0       # meters
POS_THETA = 0   # radians

MAX_RANGE = 1 # meters
SUBSAMPLE_DISTANCE = 0.1 # meters

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

def callback(data):
    global pub
    # rospy.loginfo("pos: %lf, %lf, %lf", POS_X, POS_Y, POS_THETA)
    # rospy.loginfo("I heard a scan message")
    ###############################################################################
    # convert LiDAR scan from polar to cartesian coordinates
    ###############################################################################
    # scan paramters
    scan = data.ranges
    scan_cartesian = [];
    theta = np.linspace(0, 2*np.pi, num=360)
    cartesian_points = np.array([[0,0]])
    for i in range(0, len(scan), 1): # edit the last parameter to skip measurements
        if (scan[i] != float("inf") and scan[i] < MAX_RANGE and scan[i] > 0.05):
            x = scan[i] * np.cos(theta[i])
            y = scan[i] * np.sin(theta[i])
            cartesian_points = np.concatenate((cartesian_points, [[x, y]]))
    cartesian_points = cartesian_points[1:, :]
   
    ###############################################################################
    # determine clusters
    ###############################################################################
    cluster_centers = []
    rot = np.array([[np.cos(POS_THETA), -np.sin(POS_THETA)], [np.sin(POS_THETA), np.cos(POS_THETA)]])
    for i in range(len(cartesian_points)):
        raw_point = cartesian_points[i]
        point = np.array([raw_point[0], raw_point[1]])

        # transform point to world frame
        point = rot @ point.T

        point_new = True
        for k in range(0, len(cluster_centers), 2):
            compare_point = np.array([cluster_centers[k] - POS_X, cluster_centers[k+1] - POS_Y])
            if (pow(point[0]-compare_point[0],2) + pow(point[1]-compare_point[1],2)) < pow(SUBSAMPLE_DISTANCE,2):
                point_new = False
                break
        if point_new:
            cluster_centers.append(round(point[0].item() + POS_X, 2))
            cluster_centers.append(round(point[1].item() + POS_Y, 2))

    rospy.loginfo(cluster_centers)
    
    cluster = Cluster()
    cluster.cluster_centres = cluster_centers
    pub.publish(cluster)

def ground_truth_callback(data):
    global POS_X, POS_Y, POS_THETA
    roll, pitch, yaw  = euler_from_quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
    POS_X = data.position.x
    POS_Y = data.position.y
    POS_THETA = yaw

def listener():
    rospy.init_node('listener', anonymous=False)
    rospy.Subscriber("robot_pos", Pose, ground_truth_callback, queue_size=1)
    rospy.Subscriber("scan", LaserScan, callback, queue_size=1)
    rospy.spin()

if __name__=='__main__':
    pub = rospy.Publisher('clusters', Cluster, queue_size=1)
    listener()