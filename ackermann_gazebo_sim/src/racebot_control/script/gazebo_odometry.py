#!/usr/bin/env python

'''
Translates Gazebo link states to per-robot odometry, capped at 20 Hz.
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped, Vector3
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import tf2_ros


class OdometryNode:

    def __init__(self):
        self.robot_name = rospy.get_param('~robot_name', rospy.get_namespace().strip('/') or 'racebot')
        self.link_name = self.robot_name + '::' + self.robot_name + '/base_footprint'
        self.odom_frame = self.robot_name + '/odom'
        self.base_frame = self.robot_name + '/base_footprint'

        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None

        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=1)
        self.tf_pub = tf2_ros.TransformBroadcaster()

        rospy.Timer(rospy.Duration(.05), self.timer_callback)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        try:
            idx = msg.name.index(self.link_name)
        except ValueError:
            return
        self.last_received_pose = msg.pose[idx]
        self.last_received_twist = msg.twist[idx]
        self.last_recieved_stamp = rospy.Time.now()

    def timer_callback(self, event):
        if self.last_recieved_stamp is None:
            return

        cmd = Odometry()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = self.odom_frame
        cmd.child_frame_id = self.base_frame
        cmd.pose.pose = self.last_received_pose
        cmd.twist.twist = self.last_received_twist
        cmd.pose.covariance = [1e-3, 0, 0, 0, 0, 0,
                               0, 1e-3, 0, 0, 0, 0,
                               0, 0, 1e6, 0, 0, 0,
                               0, 0, 0, 1e6, 0, 0,
                               0, 0, 0, 0, 1e6, 0,
                               0, 0, 0, 0, 0, 1e3]
        cmd.twist.covariance = [1e-9, 0, 0, 0, 0, 0,
                                0, 1e-3, 1e-9, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 1e-9]

        self.pub_odom.publish(cmd)

        tf = TransformStamped(
            header=Header(frame_id=self.odom_frame, stamp=cmd.header.stamp),
            child_frame_id=self.base_frame,
            transform=Transform(
                translation=Vector3(
                    x=cmd.pose.pose.position.x,
                    y=cmd.pose.pose.position.y,
                    z=cmd.pose.pose.position.z
                ),
                rotation=cmd.pose.pose.orientation
            )
        )
        self.tf_pub.sendTransform(tf)


if __name__ == '__main__':
    rospy.init_node('gazebo_odometry_node')
    node = OdometryNode()
    rospy.spin()
