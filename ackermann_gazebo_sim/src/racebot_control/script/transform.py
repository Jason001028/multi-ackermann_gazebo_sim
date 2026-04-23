#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist


def callback(data):
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"
    msg.drive.speed = data.linear.x
    msg.drive.acceleration = 1
    msg.drive.jerk = 1
    msg.drive.steering_angle = data.angular.z
    msg.drive.steering_angle_velocity = 1
    pub.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('nav_sim', anonymous=True)
        # publish to namespace-relative topic so each car gets its own ackermann_cmd
        pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)
        rospy.Subscriber('cmd_vel', Twist, callback, queue_size=1, buff_size=52428800)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
