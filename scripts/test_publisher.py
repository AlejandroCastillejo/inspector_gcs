#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix

def test_publisher():
    uav_gps_pos_pub = rospy.Publisher('uav_2/dji_sdk/gps_position', NavSatFix, queue_size=10)
    rospy.init_node('test_publisher')

    while not rospy.is_shutdown():
        gps_position = NavSatFix()
        gps_position.latitude = 37.2
        gps_position.longitude = -5.8
        uav_gps_pos_pub.publish(gps_position)
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        test_publisher()
    except rospy.ROSInterruptException:
        pass