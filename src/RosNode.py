#!/usr/bin/env python

import rospy


class RosNode(object):
    """Abstract class to implement a generic Node of framework ROS.

    Attributes:
        node_name(str): Name of RosNode.
        node_rate(float): Rate of loop defined by user.

    """

    def __init__(self, name, rate):
        """Set name and rate of node."""
        self.node_name = name
        self.node_rate = rate

    def run(self):
        """Virtual method that implement lifecycle of RosNode."""
        pass
