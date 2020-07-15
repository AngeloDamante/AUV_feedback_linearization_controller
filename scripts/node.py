#!/usr/bin/env python

__author__ = 'Angelo Damante'
__mail__ = 'angelo.damante16@gmail.com'

import rospy
from src.ContolNode import ControlNode

if __name__ == '__main__':
    control_node = ControlNode('control_node', 10)
    try:
        control_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('shutdown of %s requested', rospy.get_name())
