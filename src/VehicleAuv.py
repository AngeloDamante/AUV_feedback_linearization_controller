#!/usr/bin/env python

import rospy
import numpy as np
from Navigation import NAVIGATION


class VehicleAuv(object):
    """Implements the mathematical operations to calculate the dynamic model for an AUV.

    Attributes:
        M(ndarray): Inertial matrix (6x6).
        C(ndarray): Coriolis matrix (6x6).
        D(ndarray): Damping matrix (6x6).
        g(ndarray): Vector of gravity and buoyancy expressed in body-fixed frame (6x1).
        buoyancy(float): Constant of buoyancy.
        cob(ndarray): Vector representative the center of buoyancy expressed in earth-fixed frame (3x1).
        origin(ndarray): Vector of initial origin expressed in lla (3x1).

    """

    def __init__(self):
        """Initializes the attributes and load the static parameters."""
        self.M = np.zeros((6, 6))
        self.C = np.zeros((6, 6))
        self.D = np.zeros((6, 6))
        self.D_coeff = np.zeros((6, 6))
        self.g = np.zeros((6, 1))
        self.buoyancy = 0.0
        self.cob = np.zeros((3, 1))
        self.origin = np.zeros((3, 1))

        self.load_static_parameters()

    def load_static_parameters(self):
        """Method that load parameters (M, D, Buoyancy, cob) from vehicle_model.launch file."""

        # Buoyancy
        self.buoyancy = rospy.get_param("/vehicle_model/buoyancy")

        # Center of buoyancy
        self.cob[0] = rospy.get_param("/vehicle_model/center_of_buoyancy/x")
        self.cob[1] = rospy.get_param("vehicle_model/center_of_buoyancy/y")
        self.cob[2] = rospy.get_param("vehicle_model/center_of_buoyancy/z")

        # Damping matrix semplified
        self.D_coeff[0] = rospy.get_param("/vehicle_model/d_coeff/x")
        self.D_coeff[1] = rospy.get_param("/vehicle_model/d_coeff/y")
        self.D_coeff[2] = rospy.get_param("/vehicle_model/d_coeff/z")
        self.D_coeff[3] = rospy.get_param("/vehicle_model/d_coeff/phi")
        self.D_coeff[4] = rospy.get_param("/vehicle_model/d_coeff/theta")
        self.D_coeff[5] = rospy.get_param("/vehicle_model/d_coeff/psi")

        # Inertial matrix
        inv_m = np.zeros((6, 6))
        inv_m[0, 0] = rospy.get_param("/vehicle_model/inv_m/x")
        inv_m[1, 1] = rospy.get_param("/vehicle_model/inv_m/y")
        inv_m[2, 2] = rospy.get_param("/vehicle_model/inv_m/z")
        inv_m[3, 3] = rospy.get_param("/vehicle_model/inv_m/phi")
        inv_m[4, 4] = rospy.get_param("/vehicle_model/inv_m/theta")
        inv_m[5, 5] = rospy.get_param("/vehicle_model/inv_m/psi")
        self.M = np.linalg.inv(inv_m)

        # init pose in LLA
        self.origin[0] = rospy.get_param("/vehicle_model/init_pose/lat")
        self.origin[1] = rospy.get_param("/vehicle_model/init_pose/lon")
        self.origin[2] = rospy.get_param("/vehicle_model/init_pose/alt")

    def update_dynamic_parameters(self, rpy, ni):
        """Implements mathematical operations to compute 'C' and 'g' matrix.

        This method update dynamic variables of vehicle that are used to compute dynamic model.

        Args:
            rpy(ndaray): Vector of body(vehicle) orientation in earth-fixed frame (3x1).
            ni(ndarray): Vector of linear/angular velocity in the body-fixed frame (6x1).

        """
        m = self.M[0, 0]
        Ixx, Iyy, Izz = self.M[3, 3], self.M[4, 4], self.M[5, 5]
        u, v, w, p, q, r = ni[0], ni[1], ni[2], ni[3], ni[4], ni[5]

        # Coriolis matrix
        self.C[0, 4] = m * w
        self.C[0, 5] = -m * v
        self.C[1, 3] = -m * w
        self.C[1, 5] = m * u
        self.C[2, 3] = m * v
        self.C[2, 4] = -m * u
        self.C[3, 1] = m * w
        self.C[3, 2] = -m * v
        self.C[3, 4] = Izz * r
        self.C[3, 5] = -Iyy * q
        self.C[4, 0] = -m * w
        self.C[4, 2] = m * u
        self.C[4, 3] = -Izz * r
        self.C[4, 5] = Ixx * p
        self.C[5, 0] = m * v
        self.C[5, 1] = -m * u
        self.C[5, 3] = Iyy * q
        self.C[5, 4] = -Ixx * p

        # Damping matrix
        np.fill_diagonal(self.D, self.D_coeff * np.abs(ni))

        # g matrix with neutrum AUV robot
        x_B, y_B, z_B = self.cob[0], self.cob[1], self.cob[2]
        roll, pitch, yaw = rpy[0], rpy[1], rpy[2]
        self.g[3] = self.buoyancy * (y_B * np.cos(pitch) * np.cos(roll) - z_B * np.cos(pitch) * np.sin(roll))
        self.g[4] = self.buoyancy * (x_B * np.cos(pitch) * np.cos(roll) - z_B * np.sin(pitch))
        self.g[5] = self.buoyancy * (x_B * np.cos(pitch) * np.cos(roll) + y_B * np.sin(pitch))

    def get_dynamic_model(self):
        """Return the four updated matrix of vehicle model.

        Returns:
            M(ndarray 6x6), C(ndarray 6x6), D(ndarray 6x6), g(ndarray 6x1).

        """
        return self.M, self.C, self.D, self.g

    def ned2body_linear(self, ned, rpy_angles):
        """Linear transformation of quantity expressed in earth-fixed frame to quantity expressed in body-fixed frame.

        Args:
            ned(ndarray): Vector expressed in earth-fixed frame (3x1).
            rpy_angles(ndarray): Vector of Euler angles (3x1).

        Returns:
            body(ndarray): Vector expressed in body-fixed frame (3x1).

        """
        R = self.rotation_matrix(rpy_angles[0], rpy_angles[1], rpy_angles[2])
        body = np.dot(R, ned)

        return body

    def ned2body_angular(self, ned, rpy_angles):
        """Angular transformation of quantity expressed in earth-fixed frame to quantity expressed in body-fixed frame.

        Args:
            ned(ndarray): Vector expressed in earth-fixed frame (3x1).
            rpy_angles(ndarray): Vector of Euler angles (3x1).

        Returns:
            body(ndarray): Vector expressed in body-fixed frame (3x1).

        """
        Jo = self.jacobian_angular(rpy_angles[0], rpy_angles[1])
        body = np.dot(Jo, ned)

        return body

    def body2ned_linear(self, body, rpy_angles):
        """Linear transformation of quantity expressed in body-fixed frame to quantity expressed in earth-fixed frame.

        Args:
            body(ndarray): Vector expressed in body-fixed frame (3x1).
            rpy_angles(ndarray): Vector of Euler angles (3x1).

        Returns:
            ned(ndarray): Vector expressed in earth-fixed frame (3x1).

        """
        Rt = np.transpose(self.rotation_matrix(rpy_angles[0], rpy_angles[1], rpy_angles[2]))
        ned = np.dot(Rt, body)

        return ned

    def body2ned_angular(self, body, rpy_angles):
        """Angular transformation of quantity expressed in body-fixed frame to quantity expressed in earth-fixed frame.

        Args:
            body(ndarray): Vector expressed in body-fixed frame (3x1).
            rpy_angles(ndarray): Vector of Euler angles (3x1).

        Returns:
            ned(ndarray): Vector expressed in earth-fixed frame (3x1).

        """
        Jo_inv = np.linalg.inv(self.jacobian_angular(rpy_angles[0], rpy_angles[1]))
        ned = np.dot(Jo_inv, body)

        return ned

    def jacobian_angular(self, roll, pitch):
        """Compute Jacobian that maps angular velocity from earth-fixed frame to body-fixed frame.

        Returns:
            Jacobian angular matrix (3x3).

        """
        Jo = np.zeros((3, 3))

        Jo[0, 0] = 1
        Jo[0, 2] = -np.sin(pitch)
        Jo[1, 1] = np.cos(roll)
        Jo[1, 2] = np.cos(pitch) * np.sin(roll)
        Jo[2, 1] = -np.sin(roll)
        Jo[2, 2] = np.cos(pitch) * np.cos(roll)

        return Jo

    def rotation_matrix(self, roll, pitch, yaw):
        """Compute rotation matrix that maps unit vectors from earth-fixed frame to body-fixed frame.

        Returns:
            Rotation matrix (3x3).

        """
        RPY = np.zeros((3, 3))

        RPY[0, 0] = np.cos(yaw) * np.cos(pitch)
        RPY[0, 1] = np.sin(yaw) * np.cos(pitch)
        RPY[0, 2] = -np.sin(pitch)
        RPY[1, 0] = -(np.sin(yaw) * np.cos(roll)) + (np.cos(yaw) * np.sin(pitch) * np.sin(roll))
        RPY[1, 1] = (np.cos(yaw) * np.cos(roll)) + (np.sin(yaw) * np.sin(pitch) * np.sin(roll))
        RPY[1, 2] = np.sin(roll) * np.cos(pitch)
        RPY[2, 0] = (np.sin(yaw) * np.sin(roll)) + (np.cos(yaw) * np.sin(pitch) * np.cos(roll))
        RPY[2, 1] = (-np.cos(yaw) * np.sin(roll)) + (np.sin(yaw) * np.sin(pitch) * np.cos(roll))
        RPY[2, 2] = np.cos(roll) * np.cos(pitch)

        return RPY

    def lla2body(self, lla, rpy_angles):
        """Compute conversion of vector expressed in lla to vector expressed in body-fixed frame.

        Returns:
            body: ndarray in body-fixed frame.

        """
        navigation = NAVIGATION()
        ned = navigation.lla2ned(lla, self.origin)
        body = self.ned2body_linear(ned, rpy_angles)
        return body
