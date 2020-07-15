#!/usr/bin/env python

import numpy as np
from PI import PI
from VehicleAuv import VehicleAuv


class InverseDynamicController(object):
    """Implements the mathematical operations for Inverse Dynamic Controller.

    Attributes:
        vehicle(obj:VehicleAUV): Vehicle to be controlled.
        hat_M(ndarray): Estimate of Inertial matrix (6x6).
        hat_C(ndarray): Estimate of Coriolis matrix (6x6).
        hat_D(ndarray): Estimate of Dumping matrix (6x6).
        hat_g(ndarray): Estimate of g matrix (6x1).
        Kp(ndarray): Proportional gains constant diagonal matrix (6x6).
        Ki(ndarray): Integral gains constant diagonal matrix (6x6).
        integral_guard(ndarray): Integral guard for PI (6x1).
        PI(PI): PI controller initialized of Kp, Ki and integral_guard.
        tau_saturation(ndarray): Values of saturation of tau (6x1).

    """

    def __init__(self):
        """Initalizes gain matrices and static parameters of controller."""

        # Vehicle model
        self.vehicle = VehicleAuv()
        self.hat_M = np.zeros((6, 6))
        self.hat_C = np.zeros((6, 6))
        self.hat_D = np.zeros((6, 6))
        self.hat_g = np.zeros((6, 1))

        # PI
        self.Kp = np.zeros((6, 6))
        self.Ki = np.zeros((6, 6))
        self.load_gains()
        self.integral_guard = np.array([40, 10, 40, 0, 0, 20]).reshape((6, 1))
        self.PI = PI(self.Kp, self.Ki, self.integral_guard)

        # saturation values
        self.tau_saturation = np.array([60.0, 10, 60.0, 0.0, 0.0, 20.0]).reshape((6, 1))

    def load_gains(self):
        """Load the proportional and integral gains from gains vector chosed with Fossen documentation."""
        gains = np.array([3, 3, 2, 0, 0, 3]).reshape((6, 1))
        np.fill_diagonal(self.Kp, 2 * gains)
        np.fill_diagonal(self.Ki, gains * gains)

    def control_law(self, rpy, ni, ni_ref_dot):
        """Compute of inverse dynamics control action.

        Sequence of operations:
            1. Get the four matrices of dynamic model of vehicle.
            2. Compute ni_dot as my 'y' of control.
            3. Compute of control law.

        Args:
            rpy(ndarray), ni(ndarray): physical quantities to compute dynamic parameters for vehicle.
            ni_ref_dot(ndarray): quantity for y.

        Returns:
            Control law: tau(6x1).

        """
        self.vehicle.update_dynamic_parameters(rpy, ni)
        self.hat_M, self.hat_C, self.hat_D, self.hat_g = self.vehicle.get_dynamic_model()

        # y vector is ni_dot
        y = ni_ref_dot - self.PI.getResult()
        tau = np.dot(self.hat_M, y) + np.dot(self.hat_C, ni) + np.dot(self.hat_D, ni) + self.hat_g

        # saturation of tau
        tau = np.clip(tau, -self.tau_saturation, self.tau_saturation)
        return tau.astype('float32')
