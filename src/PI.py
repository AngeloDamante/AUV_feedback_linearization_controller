#!/usr/bin/env python

import rospy
import numpy as np


class PI(object):
    """Class to implement PI control.

    The PI is handle from control node and used by controller class.

    Attributes:
        Kp(ndarray): proportional gains.
        Ki(ndarray): integral gains.
        integral_guard(ndarray): saturation level of integral.

        integral_term(ndarray): integral term with memory of values.
        integral_value(ndarray): integral value with integral gains of PI.
        proportional_value(ndarray): proportional value with proportional gains of PI.
        result(ndarray): result of PI value.

    """

    def __init__(self, Kp, Ki, integral_guard):
        """
        Args:
            Kp: ndarray with proportional gains.
            Ki: ndarray with integral gains.
            integral_guard: ndarray with saturation levels.

        """
        self.Kp = Kp
        self.Ki = Ki
        self.integral_guard = integral_guard

        # array like integral_guard
        self.integral_term = np.zeros((6, 1))
        self.integral_value = np.zeros((6, 1))
        self.proportional_value = np.zeros((6, 1))
        self.result = np.zeros((6, 1))

    def update(self, value, dt):
        """Update the values of PI.

        Args:
            value: error value for PI.

        """
        self.integral_term += value * dt
        self.integral_value = np.clip(np.dot(self.Ki, self.integral_term), -self.integral_guard, self.integral_guard)
        self.proportional_value = np.dot(self.Kp, value)
        self.result = self.integral_value + self.proportional_value

    def reset(self):
        """To reset integral part of PI when start new mission (or submission)."""
        self.integral_term = np.zeros((6, 1))

    def getResult(self):
        """Get function.

        Returns:
            result: ndarray with result of PI

        """
        return self.result
