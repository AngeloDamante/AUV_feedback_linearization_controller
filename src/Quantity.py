ò#!/usr/bin/env python

import rospy


class Quantity(object):
    """Object to handle various paramenters of physical quantity.

    Attributes:
        value(ndarray): actual value.
        last_value(ndarray): value in last call.
        last_sampling(rospy.Time): sampling time in last call.
        dot(ndarray): derivative of physical quantity.

    """

    def __init__(self, value):
        self.value = value
        self.last_value = value
        self.last_sampling = rospy.Time(0)
        self.dot = value
