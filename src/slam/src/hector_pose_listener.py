#!/usr/bin/env python

# ros imports
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from gazebo_msgs.msg import ModelStates # for ground truth in Gazebo simulation

ROBOT_INDEX = 1 # gazebo simulation robot index
POSE_TRUTH = Pose() # ground truth

# save the ground turth from the gazebo simulation
def callback_truth(data):
    global POSE_TRUTH
    POSE_TRUTH = data.pose[ROBOT_INDEX]

# save the SLAM pose estimate
def callback_estimate(data):
    rate = rospy.Rate(5) # set update rate to 5Hz
    robot_pos = Pose()
    robot_pos = data.pose
    pub_estimate.publish(robot_pos)
    pub_truth.publish(POSE_TRUTH)
    rate.sleep()

# setup the subscribers
def robot_pos():
    rospy.init_node('robot_pos', anonymous=True)
    rospy.Subscriber("gazebo/model_states", ModelStates, callback_truth, queue_size=1)
    rospy.Subscriber("slam_out_pose", PoseStamped, callback_estimate, queue_size=1)
    rospy.spin()

# setup the state estimate and state ground truth publishers
if __name__=='__main__':
    pub_estimate = rospy.Publisher('robot_pos', Pose, queue_size=1)
    pub_truth = rospy.Publisher('robot_pos_ground_truth', Pose, queue_size=1)
    robot_pos()