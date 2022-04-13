#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @author Tim Bohne

import rospy
from std_msgs.msg import String, Bool

from execution_monitoring import config


class PowerManagementMonitoring:
    """
    Provides monitoring solutions for power management failures.
    """

    def __init__(self):
        self.active_catastrophe_monitoring = True
        self.active_contingency_monitoring = True
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)
        self.aggravate_pub = rospy.Publisher('/aggravate', String, queue_size=1)
        self.interrupt_reason_pub = rospy.Publisher('/interrupt_reason', String, queue_size=1)
        rospy.Subscriber('/watchdog', String, self.watchdog_callback, queue_size=1)
        rospy.Subscriber('/fully_charged', String, self.fully_charged_callback, queue_size=1)
        rospy.Subscriber('/catastrophe_launched', String, self.catastrophe_callback, queue_size=1)
        rospy.Subscriber('/resolve_power_management_failure_success', Bool, self.resolution_callback, queue_size=1)

    def resolution_callback(self, msg):
        """
        Resolver communication callback - determines whether resolution attempt was successful.
        Aggravates to catastrophe in case of failure.

        @param msg: callback message - whether resolution of power management failure was successful
        """
        if not msg.data:
            self.interrupt_reason_pub.publish(config.POWER_MANAGEMENT_CATA)
            self.aggravate_pub.publish(config.POWER_MANAGEMENT_CATA)

    def catastrophe_callback(self, msg):
        """
        Callback that provides information about launched catastrophes - deactivates active monitoring.

        @param msg: callback message
        """
        self.active_catastrophe_monitoring = False

    def fully_charged_callback(self, msg):
        """
        Callback that provides information about a fully charged battery.
        Reactivates monitoring for contingencies and catastrophes.

        @param msg: callback message
        """
        self.active_catastrophe_monitoring = True
        self.active_contingency_monitoring = True

    def watchdog_callback(self, msg):
        """
        Callback that receives the information provided by the battery watchdog module.

        Important aspects:
        - detected contingency cases -> immediately return to base
        - detected catastrophe cases -> no longer possible to reach the base

        @param msg: callback message - rating for the situation, e.g., "CONT", "CATO", "NORM"
        """
        if self.active_contingency_monitoring and msg.data == config.CONTINGENCY_MSG:
            rospy.loginfo("battery watchdog module signals contingency -- initiate contingency")
            rospy.loginfo(config.POWER_MANAGEMENT_FAILURES[0])
            self.contingency_pub.publish(config.POWER_MANAGEMENT_FAILURES[0])
            self.active_contingency_monitoring = False
        elif self.active_catastrophe_monitoring and msg.data == config.CATASTROPHE_MSG:
            rospy.loginfo("battery watchdog module signals catastrophe -- initiate catastrophe")
            rospy.loginfo(config.POWER_MANAGEMENT_CATA)
            self.catastrophe_pub.publish(config.POWER_MANAGEMENT_CATA)


def node():
    """
    Power management monitoring node.
    """
    rospy.init_node('power_management_monitoring')
    # sleep for some time -- initially the watchdog module is not really reliable
    rospy.sleep(config.INITIAL_SLEEP_TIME)
    rospy.loginfo("launch power management monitoring..")
    PowerManagementMonitoring()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
