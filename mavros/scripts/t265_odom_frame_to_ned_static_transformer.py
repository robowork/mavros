#!/usr/bin/env python3

import tf
import tf2_ros
import tf2_geometry_msgs as tg
import rospy
import copy
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
import sys
import numpy as np

class StaticTransformer:
    def __init__(self):
        # transform camera_odom_sample odometry to world_ned
        # task find body_frame,
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.input_topic = '/camera/odom/sample'
        self.output_topic = '/mavros/local_position/ned_pose'
        self.body_frame_name = 'base_link'
        self.world_frame_name = 'world_ned'

        # setup publishers
        self.ned_pose_pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=10)

    def transform_frame(self):
        try:
            txn = self.tf_buffer.lookup_transform_full(\
                  target_frame=self.world_frame_name,
                  target_time=rospy.Time(),
                  source_frame=self.body_frame_name,
                  source_time=rospy.Time(),
                  fixed_frame=self.world_frame_name)

            body_pose = PoseStamped()
            body_pose.header = txn.header
            body_pose.pose.position.x = txn.transform.translation.x
            body_pose.pose.position.y = txn.transform.translation.y
            body_pose.pose.position.z = txn.transform.translation.z
            body_pose.pose.orientation = txn.transform.rotation

            self.ned_pose_pub.publish(body_pose)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,\
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(10,e)


if __name__ == '__main__':
    rospy.init_node('t265_odom_frame_to_ned', anonymous=True)
    transform_publisher = StaticTransformer()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            transform_publisher.transform_frame()
        except KeyboardInterrupt:
            rospy.loginfo("received keyboard interrupt")
            sys.exit(0)
        rate.sleep()
