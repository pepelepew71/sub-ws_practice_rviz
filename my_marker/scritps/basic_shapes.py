#!/usr/bin/env python

"""
Showing marker with random color and shape.
"""

from __future__ import print_function
from __future__ import division

import random

import rospy
from visualization_msgs.msg import Marker

def _get_basic_marker():
    m = Marker()
    m.header.frame_id = "/map"
    m.header.stamp = rospy.Time.now()
    m.ns = "basic_shapes"
    m.id = 0
    m.action = m.ADD
    m.scale.x = 0.1
    m.scale.y = 0.1
    m.scale.z = 0.1
    m.color.a = 1.0
    m.pose.position.x = random.random()*2.0 - 1.0
    m.pose.position.y = random.random()*2.0 - 1.0
    m.pose.orientation.w = 1.0
    return m

def _set_color(m):
    choices = {0: "r", 1: "g", 2: "b"}
    color = choices[random.randint(a=0, b=2)]
    if color == "r":
        m.color.r = 1.0
    elif color == "g":
        m.color.g = 1.0
    elif color == "b":
        m.color.b = 1.0
    else:
        rospy.loginfo(msg="no matching color")
    return m

def _set_shape(m):
    choices = {0: m.ARROW, 1: m.CUBE, 2: m.SPHERE, 3: m.CYLINDER}
    m.type = choices[random.randint(a=0, b=3)]
    return m

def get_marker():
    m = _get_basic_marker()
    m = _set_color(m)
    m = _set_shape(m)
    return m

if __name__ == "__main__":

    rospy.init_node(name="basic_shapes", anonymous=False)
    pub = rospy.Publisher(name="visualization_marker", data_class=Marker, queue_size=1)

    rate = rospy.Rate(hz=1)

    while not rospy.is_shutdown():
        marker = get_marker()
        pub.publish(marker)
        rate.sleep()
