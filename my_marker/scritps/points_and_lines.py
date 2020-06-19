#!/usr/bin/env python

"""
Animation of using points and line_strip marker
"""

from __future__ import print_function
from __future__ import division

from math import cos, sin, pi

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def get_marker_points():
    m = Marker()
    m.header.frame_id = "/map"
    m.header.stamp = rospy.Time.now()
    m.ns = "points_and_lines"
    m.id = 0
    m.action = m.ADD
    m.type = m.POINTS
    m.pose.orientation.w = 1.0
    m.scale.x = 0.02
    m.scale.y = 0.02
    m.color.g = 1.0
    m.color.a = 1.0
    return m

def get_marker_line_strip():
    m = Marker()
    m.header.frame_id = "/map"
    m.header.stamp = rospy.Time.now()
    m.ns = "points_and_lines"
    m.id = 1
    m.action = m.ADD
    m.type = m.LINE_STRIP
    m.pose.orientation.w = 1.0
    m.scale.x = 0.01
    m.color.b = 1.0
    m.color.a = 1.0
    return m

if __name__ == "__main__":

    rospy.init_node(name="points_and_lines", anonymous=False)
    pub = rospy.Publisher(name="visualization_marker", data_class=Marker, queue_size=10)

    marker_points = get_marker_points()
    marker_line_strip = get_marker_line_strip()

    f = 0.0
    rate = rospy.Rate(hz=10)

    while not rospy.is_shutdown():

        marker_points.points = list()
        marker_line_strip.points = list()

        for i in range(101):
            p = Point()
            p.x = 1.0 * sin(f + float(i) / 100.0 * 2.0 * pi)
            p.y = 1.0 * cos(f + float(i) / 100.0 * 2.0 * pi)
            p.z = float(i)/100.0
            marker_points.points.append(p)
            marker_line_strip.points.append(p)

        pub.publish(marker_points)
        pub.publish(marker_line_strip)

        f += 0.04

        rate.sleep()
