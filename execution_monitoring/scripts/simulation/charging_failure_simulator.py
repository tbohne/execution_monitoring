#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @author Tim Bohne

import rospy
from std_msgs.msg import String, Float64


class ChargingFailureSimulator:
    """
    Provides simulations of charging failure cases.
    """

    def __init__(self):
        self.sim_undocking_fail = False
        self.sim_info_pub = rospy.Publisher('/sim_info', String, queue_size=1)
        rospy.Subscriber('/sim_undocking_failure', String, self.undocking_fail_callback, queue_size=1)
        rospy.Subscriber('/sim_docking_failure_raised_ramp', String, self.docking_fail_callback, queue_size=1)
        rospy.Subscriber('/charge_action', String, self.charge_callback, queue_size=1)

    def docking_fail_callback(self, msg):
        """
        Initiates simulation of a docking failure due to a raised container ramp.

        @param msg: callback message
        """
        rospy.loginfo("sim docking fail due to raised container ramp..")
        self.raise_container_ramp()

    def charge_callback(self, msg):
        """
        Is called as a result of a charge action.

        @param msg:  callback message
        """
        if self.sim_undocking_fail:
            rospy.loginfo("sim undocking fail due to raised container ramp..")
            self.sim_info_pub.publish("charging failure monitoring: sim undocking failure")
            self.raise_container_ramp()
            self.sim_undocking_fail = False

    def undocking_fail_callback(self, msg):
        """
        Initiates simulation of an undocking failure.

        @param msg: callback message
        """
        rospy.loginfo("preparing undocking fail sim..")
        self.sim_undocking_fail = True

    def raise_container_ramp(self):
        """
        Raises the ramp of the mobile base station (container).
        """
        rospy.loginfo("raising container ramp..")
        self.sim_info_pub.publish("charging failure monitoring: sim failure -- raising container ramp")
        container_pub = rospy.Publisher('/container/rampB_position_controller/command', Float64, queue_size=1)
        for _ in range(3):
            container_pub.publish(0.0)
            rospy.sleep(0.5)


def node():
    """
    Charging failure simulation node.
    """
    rospy.init_node('charging_failure_simulator')
    rospy.loginfo("launch charging failure simulator..")
    ChargingFailureSimulator()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
