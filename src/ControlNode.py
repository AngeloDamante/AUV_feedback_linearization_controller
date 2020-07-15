#!/usr/bin/env python

__author__ = 'Angelo Damante'
__mail__ = 'angelo.damante16@gmail.com'

import rospy
import numpy as np
from threading import RLock
from RosNode import RosNode
from msgs.msg import Tau, Forces, Euler, Measurement
from msgs.msg import Imu, Dvl, Speed, Position, Trackers
from msgs.msg import PhysicalQuantity as PQ
from std_msgs.msg import String
from InverseDynamicController import InverseDynamicController
from Quantity import Quantity

from PI import PI
from copy import deepcopy


class ControlNode(RosNode):
    """Class that manages the rosnode 'control_node'.

    This node is implemented for inverse dynamic control.

    Topics:
        * Subscriber: /drivers/imu, /drivers/dvl, /reference/rpy,speed,ll,depth
        * Publisher: /control/tau, /measurement

    Attributes:
        BaseClass: node_name, node_rate
        mutex(Rlock): To handle syncronization of operations.
        controller(InverseDynamicController): Instance of control model for mathematical operations.
        ready(bool): Flag to handle call of controller.
        eta2(ndaray): Vector of body(vehicle) orientation in earth-fixed frame (3x1).
        ni(ndarray): Vector of linear/angular velocity in the body-fixed frame (6x1).
        speed_ref(ndarray): Vector of speed reference in body-fixed frame (3x1).
        eta1_ref_body(Quantity): Object that handle eta1_ref in body-fixed frame.
        eta2_ref(Quantity): Object that handle eta2_ref.
        ni_ref(Quantity): Object that handle ni_ref.
        ni_tilde(ndarray): Vector of error, compute as ni - ni_ref.
        reference_flags(dict): Dict to wait first cycle.
        node_loop(rospy.rate): To handle frequency of publications.
        pub_tau(topics.Publisher): Publisher of tau.
        pub_measurement(topics.Publisher): Publisher of measurement.

    """

    def __init__(self, name, rate):
        """Initializes ROSnode, instantiates the controller and initializes dynamic quantities (eta, ni, ...).

        Args:
            node_name(str): Name of node.
            node_rate(float): To fix rate of loop.

        """
        super(ControlNode, self).__init__(name, rate)
        self.mutex = RLock()
        self.controller = InverseDynamicController()
        self.ready = False

        # Physical quantities from sensors
        self.eta2 = np.zeros((3, 1))
        self.ni = np.zeros((6, 1))

        # References from topic
        self.speed_ref = np.zeros((3, 1))
        self.eta1_ref_body = Quantity(np.zeros((3, 1)))
        self.eta2_ref = Quantity(np.zeros((3, 1)))
        self.ni_ref = Quantity(np.zeros((6, 1)))

        # Error value
        self.ni_tilde = np.zeros((6, 1))

        # flags to wait first cycle
        self.reference_flags = {'ll': False, 'rpy': False, 'depth': False}

        # ROS
        rospy.init_node(self.node_name, anonymous=False)
        self.node_loop = rospy.Rate(self.node_rate)
        self.StartSubscriptions()
        self.pub_tau = rospy.Publisher('/control/tau', Tau, queue_size=1)
        self.pub_measurement = rospy.Publisher('/measurement', Measurement, queue_size=1)

    def StartSubscriptions(self):
        """Subscribe node to topics."""
        rospy.Subscriber('/drivers/dvl', Dvl, self.dvl_callback)
        rospy.Subscriber('/drivers/imu', Imu, self.imu_callback)
        rospy.Subscriber('/reference/depth', Position, self.refDepth_callback)
        rospy.Subscriber('/reference/speed', Speed, self.refSpeed_callback)
        rospy.Subscriber('/reference/rpy', Euler, self.refRpy_callback)
        rospy.Subscriber('/reference/ll', Position, self.refLL_callback)
        rospy.Subscriber('/control/trackers_enabled', Trackers, self.trackersControl_callback)

    def trackersControl_callback(self, msg):
        """Entering this callback means that a new mission has started. The PI need to be resetted.

        NewMissionReceived(msg)

        """
        self.mutex.acquire()
        if ('rpy_tracker' in msg.trackers) and ('speed_tracker' in msg.trackers) and ('depth_tracker' in msg.trackers):
            self.controller.PI.reset()
            self.ready = False

        if ('rpy_tracker' in msg.trackers) and ('ll_tracker' in msg.trackers) and ('depth_tracker' in msg.trackers):
            self.controller.PI.reset()
            self.ready = False
        self.mutex.release()

    def imu_callback(self, msg):
        """Callback function to extract values from topic '/drivers/imu'.

        The values are:
            * angular_rate: angular velocity in body-fixed frame.
            * orientation: angular position in earth-fixed frame.

        """
        self.mutex.acquire()

        self.ni[3] = msg.angular_rate.x
        self.ni[4] = msg.angular_rate.y
        self.ni[5] = msg.angular_rate.z

        self.eta2[0] = msg.orientation.roll
        self.eta2[1] = msg.orientation.pitch
        self.eta2[2] = msg.orientation.yaw

        self.mutex.release()
        rospy.loginfo("%s receive imu", self.node_name)

    def dvl_callback(self, msg):
        """Callback function to extract values from topic '/drivers/dvl'.

        The values are:
            * velocity_instrument: linear velocity in body-fixed frame.

        """
        self.mutex.acquire()

        self.ni[0] = msg.velocity_instrument.x
        self.ni[1] = msg.velocity_instrument.y
        self.ni[2] = msg.velocity_instrument.z

        self.mutex.release()
        rospy.loginfo("%s receive dvl", self.node_name)

    def refDepth_callback(self, msg):
        """Callback function to extract values from topic '/reference/depth'.

        This method compute derivative of eta1_ref in body_fixed frame.

        The values are:
            * latitude, longitude, depth: linear components of eta reference.

        """
        self.mutex.acquire()
        depth_ref = np.array([0, 0, msg.depth]).reshape((3, 1))

        if not (self.reference_flags['depth']):
            # first assignment
            self.eta1_ref_body.last_value = self.controller.vehicle.ned2body_linear(deepcopy(depth_ref), self.eta2)
            self.eta1_ref_body.last_sampling = rospy.Time.now()
            self.reference_flags['depth'] = True
        else:
            self.eta1_ref_body.value = self.controller.vehicle.ned2body_linear(depth_ref, self.eta2)
            dt = rospy.Time.now() - self.eta1_ref_body.last_sampling

            # compute derivative
            self.eta1_ref_body.dot = (self.eta1_ref_body.value - self.eta1_ref_body.last_value) / dt.to_sec()
            self.eta1_ref_body.last_value = deepcopy(self.eta1_ref_body.value)
            self.eta1_ref_body.last_sampling = rospy.Time.now()

        self.mutex.release()
        rospy.loginfo("%s receive depth reference", self.node_name)

    def refSpeed_callback(self, msg):
        """Callback function to extract values from topic '/reference/speed'.

        The values are:
            * vx, vy, vz: linear components of ni referene.

        """
        self.mutex.acquire()

        self.speed_ref[0] = msg.vx
        self.speed_ref[1] = msg.vy
        self.speed_ref[2] = msg.vz

        self.mutex.release()
        rospy.loginfo("%s receive speed reference", self.node_name)

    def refRpy_callback(self, msg):
        """Callback function to extract values from topic '/reference/rpy'.

        This method compute derivative of eta2_ref.

        The values are:
            * roll, pitch, yaw: angular components of eta reference.

        """
        self.mutex.acquire()
        rpy_ref = np.array([msg.roll, msg.pitch, msg.yaw]).reshape((3, 1))

        if not (self.reference_flags['rpy']):
            # first assignment
            self.eta2_ref.last_value = deepcopy(rpy_ref)
            self.eta2_ref.last_sampling = rospy.Time.now()
            self.reference_flags['rpy'] = True
        else:
            dt = rospy.Time.now() - self.eta2_ref.last_sampling
            self.eta2_ref.value = rpy_ref

            # normalization of angles
            diff = self.eta2_ref.value - self.eta2_ref.last_value
            diff[0] = np.arctan2(np.sin(diff[0]), np.cos(diff[0]))
            diff[1] = np.arctan2(np.sin(diff[1]), np.cos(diff[1]))
            diff[2] = np.arctan2(np.sin(diff[2]), np.cos(diff[2]))

            # saturation of jump due step function
            if (diff[2] > 0.5 or diff[2] < -0.5):
                diff = np.zeros((3, 1))
                self.eta2_ref.value = deepcopy(self.eta2_ref.last_value)

            # compute derivative
            self.eta2_ref.dot = diff / dt.to_sec()
            self.eta2_ref.last_value = deepcopy(self.eta2_ref.value)
            self.eta2_ref.last_sampling = rospy.Time.now()

        self.mutex.release()
        rospy.loginfo("%s receive rpy reference", self.node_name)

    def refLL_callback(self, msg):
        """Callback function to extract values from topic '/reference/ll'.

        The values are:
            * latitude, longitude

        """
        self.mutex.acquire()

        ll_ref = np.array([msg.latitude, msg.longitude, 0]).reshape((3, 1))
        self.reference_flags['ll'] = True

        self.mutex.release()
        rospy.loginfo("%s receive ll reference", self.node_name)

    def publish(self, tau):
        """Publish control law in '/control/tau' topic."""

        # tau message
        sender = String('')
        tau1 = Forces(tau[0], tau[1], tau[2])
        tau2 = Euler(tau[3], tau[4], tau[5])
        tau = Tau(sender, tau1, tau2)

        self.pub_tau.publish(tau)
        rospy.loginfo(tau)

    def tester(self, tau):
        """Define message for topic '/measurement'."""

        # tau message
        sender = String('')
        tau1 = Forces(tau[0], tau[1], tau[2])
        tau2 = Euler(tau[3], tau[4], tau[5])
        tau = Tau(sender, tau1, tau2)

        # PQ message
        ni_ref = PQ(self.ni_ref.value[0],
                    self.ni_ref.value[1],
                    self.ni_ref.value[2],
                    self.ni_ref.value[3],
                    self.ni_ref.value[4],
                    self.ni_ref.value[5])
        ni = PQ(self.ni[0], self.ni[1], self.ni[2], self.ni[3], self.ni[4], self.ni[5])
        ni_tilde = PQ(self.ni_tilde[0],
                      self.ni_tilde[1],
                      self.ni_tilde[2],
                      self.ni_tilde[3],
                      self.ni_tilde[4],
                      self.ni_tilde[5])
        yaw_tilde = self.eta2_ref.value - self.eta2
        yaw_tilde = Euler(yaw_tilde[0], yaw_tilde[1], yaw_tilde[2])

        # measurement message
        measurement = Measurement(ni_ref, ni, ni_tilde, tau, yaw_tilde)
        self.pub_measurement.publish(measurement)

    def run(self):
        """Connects node to topics for publish tau and extract dynamic parameters.

        The node supplies all quantities necessary for controller to compute the control law:
            * Compute ni_ref from eta1_ref_body, speed_ref and eta2_ref.
            * Compute ni_ref_dot.
            * Compute ni_tilde with difference between ni and ni_ref.
            * Set PI of controller.

        When all the quantities are calculated, the node is ready to call the controller.

        Sequence of operations:
            1. Extract dynamic parameters (eta, ni, references and trackers) through callbacks.
            2. Compute the control law with dynamic parameters updated.
            3. Publish control law.

        """
        old_sampling = rospy.Time(0)
        while not rospy.is_shutdown():
            self.mutex.acquire()
            reference_received = all(self.reference_flags.values())
            if reference_received:
                if not self.ready:
                    # first value of ni_ref
                    self.ni_ref.last_value[0:3] = self.eta1_ref_body.dot + self.speed_ref
                    self.ni_ref.last_value[3:6] = self.controller.vehicle.ned2body_angular(self.eta2_ref.dot, self.eta2)
                    self.ni_ref.last_sampling = rospy.Time.now()

                    # error
                    old_sampling = rospy.Time.now()

                    # Node is ready to call controller
                    self.ready = True
                else:
                    # Set ni_ref
                    self.ni_ref.value[0:3] = self.eta1_ref_body.dot + self.speed_ref
                    self.ni_ref.value[3:6] = self.controller.vehicle.ned2body_angular(self.eta2_ref.dot, self.eta2)
                    dt = rospy.Time.now() - self.ni_ref.last_sampling

                    # compute derivative of ni_ref
                    self.ni_ref.dot = (self.ni_ref.value - self.ni_ref.last_value) / dt.to_sec()
                    self.ni_ref.last_value = deepcopy(self.ni_ref.value)
                    self.ni_ref.last_sampling = rospy.Time.now()

                    # Set PI of controller with error value
                    self.ni_tilde = self.ni - self.ni_ref.value
                    dt = rospy.Time.now() - old_sampling
                    self.controller.PI.update(self.ni_tilde, dt.to_sec())

                    # compute tau with eta2, ni and ni_ref_dot
                    tau = self.controller.control_law(self.eta2, self.ni, self.ni_ref.dot)

                    # publish messages
                    self.publish(tau)
                    self.tester(tau)

            self.mutex.release()
            self.node_loop.sleep()
