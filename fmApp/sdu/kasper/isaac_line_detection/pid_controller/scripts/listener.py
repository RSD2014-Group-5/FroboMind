#!/usr/bin/env python
import rospy
from pid_controller.msg import line_points

def callback(data):
    rospy.loginfo("%f y: %f" % (data.x1, data.y1))


def listener():
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber("rsd_camera/line_points", line_points, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
