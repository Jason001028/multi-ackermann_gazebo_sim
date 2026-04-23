#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped


def make_publishers(ns):
    return {
        'lrw': rospy.Publisher(ns + '/left_rear_wheel_velocity_controller/command', Float64, queue_size=1),
        'rrw': rospy.Publisher(ns + '/right_rear_wheel_velocity_controller/command', Float64, queue_size=1),
        'lfw': rospy.Publisher(ns + '/left_front_wheel_velocity_controller/command', Float64, queue_size=1),
        'rfw': rospy.Publisher(ns + '/right_front_wheel_velocity_controller/command', Float64, queue_size=1),
        'lsh': rospy.Publisher(ns + '/left_steering_hinge_position_controller/command', Float64, queue_size=1),
        'rsh': rospy.Publisher(ns + '/right_steering_hinge_position_controller/command', Float64, queue_size=1),
    }


pubs = {}


def set_throttle_steer(data):
    throttle = data.drive.speed * 31.25
    steer = data.drive.steering_angle
    pubs['lrw'].publish(throttle)
    pubs['rrw'].publish(throttle)
    pubs['lfw'].publish(throttle)
    pubs['rfw'].publish(throttle)
    pubs['lsh'].publish(steer)
    pubs['rsh'].publish(steer)


def servo_commands():
    global pubs
    rospy.init_node('servo_commands', anonymous=True)

    robot_name = rospy.get_param('~robot_name', rospy.get_namespace().strip('/') or 'racebot')
    ns = '/' + robot_name

    pubs = make_publishers(ns)

    rospy.Subscriber(ns + '/ackermann_cmd', AckermannDriveStamped, set_throttle_steer)
    rospy.spin()


if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
