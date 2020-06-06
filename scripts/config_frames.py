#!/usr/bin/env python
# license removed for brevity

import sys
import rospy
import tf2_ros, tf

from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from rospy import ROSException
import tf2_geometry_msgs

target_frame = ""
source_frame = ""
evalPosePub = rospy.Publisher('/aft_mapped_eval_pose', PoseStamped, queue_size=100)
evalOdomPub = rospy.Publisher('/navsat/odometry', Odometry, queue_size=100)

def posesCallBack(pose_msg):
    global target_frame, source_frame
    eval_pose = PoseStamped()
    eval_pose.header = pose_msg.header
    tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    tf2_listener = tf2_ros.TransformListener(tfBuffer)
    transform = tfBuffer.lookup_transform(target_frame,
                source_frame, #source frame
                rospy.Time(0), #get the tf at first available time
                rospy.Duration(1.0))
    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_msg, transform)
    eval_pose.header.frame_id = target_frame
    eval_pose.pose = pose_transformed.pose
    evalPosePub.publish(eval_pose)
    # print(eval_pose)

def gpsOdomCallBack(odom_msg):
    global target_frame, source_frame
    eval_odom = Odometry()
    eval_odom = odom_msg
    # tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    # tf2_listener = tf2_ros.TransformListener(tfBuffer)
    # transform = tfBuffer.lookup_transform(target_frame,
    #             source_frame, #source frame
    #             rospy.Time(0), #get the tf at first available time
    #             rospy.Duration(1.0))
    # pose_transformed = tf2_geometry_msgs.do_transform_pose(odom_msg.pose, transform)
    eval_odom.header.frame_id = target_frame
    y_orig = odom_msg.pose.pose.position.y
    x_orig = odom_msg.pose.pose.position.x
    # orig_quat = odom_msg.pose.pose.orientation 
    # trans_quat = (0,0,1,0)
    eval_odom.pose.pose.position.x = y_orig
    eval_odom.pose.pose.position.y = -1 * x_orig
    eval_odom.pose.pose.position.z = odom_msg.pose.pose.position.z
    eval_odom.pose.pose.orientation =  odom_msg.pose.pose.orientation 
    evalOdomPub.publish(eval_odom)
    print(eval_odom.pose.pose)

if __name__ == '__main__':    
    global target_frame, source_frame
    if len(sys.argv) < 3:
        print("usage: it takes source_frame target_frame")
    else:
        rospy.init_node('transform_frames_node')
        source_frame = sys.argv[1]
        target_frame = sys.argv[2]
        rospy.Subscriber('/aft_mapped_to_init_pose', PoseStamped, posesCallBack)
        rospy.Subscriber('/odometry/gps', Odometry, gpsOdomCallBack)
        rospy.spin()    