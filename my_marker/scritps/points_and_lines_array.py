#!/usr/bin/env python

"""
Animation of using points and line_strip marker
"""

from __future__ import print_function
from __future__ import division

from math import cos, sin, pi

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def get_marker_points(ns):
    m = Marker()
    m.header.frame_id = "/map"
    m.header.stamp = rospy.Time.now()
    m.ns = ns
    m.id = 0
    m.action = m.ADD
    m.type = m.POINTS
    m.pose.orientation.w = 1.0
    m.scale.x = 0.02
    m.scale.y = 0.02
    m.color.g = 1.0
    m.color.a = 1.0
    return m

if __name__ == "__main__":

    rospy.init_node(name="points_and_lines", anonymous=False)
    pub = rospy.Publisher(name="visualization_marker_array", data_class=MarkerArray, queue_size=10)

    f = 0.0
    rate = rospy.Rate(hz=10)
    marker_array = MarkerArray()

    while not rospy.is_shutdown():

        points1 = get_marker_points(ns="ns1")
        points2 = get_marker_points(ns="ns2")

        for i in range(101):
            x = 1.0 * sin(f + float(i) / 100.0 * 2.0 * pi)
            y = 1.0 * cos(f + float(i) / 100.0 * 2.0 * pi)
            z = float(i)/100.0

            p1 = Point()
            p1.x = x
            p1.y = y
            p1.z = z

            p2 = Point()
            p2.x = x + 2.0
            p2.y = y
            p2.z = z

            points1.points.append(p1)
            points2.points.append(p2)

        marker_array.markers.append(points1)
        marker_array.markers.append(points2)

        pub.publish(marker_array)

        f += 0.04

        rate.sleep()
