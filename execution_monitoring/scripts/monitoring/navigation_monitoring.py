#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @author Tim Bohne

import rospy
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from geopy import distance
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Bool

from execution_monitoring import config


class NavigationMonitoring:
    """
    Provides monitoring solutions for navigation failures.

    Specifically:
    - explicit move_base / move_base_flex failures
    - implicit navigation failures -- "sustained recoveries"
    """

    def __init__(self):
        self.nav_status = None
        self.status_before = None
        self.active_monitoring = True
        self.robot_pos = None
        self.robot_pos_when_started_recovery = None
        self.recovery_cnt = 0
        self.recovery_attempts = 0

        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.aggravate_pub = rospy.Publisher('/aggravate', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.resolution_failure_pub = rospy.Publisher('/resolution_failure', String, queue_size=1)
        self.interrupt_reason_pub = rospy.Publisher('/interrupt_reason', String, queue_size=1)

        rospy.Subscriber(config.GOAL_STATUS_TOPIC, GoalStatusArray, self.nav_status_callback, queue_size=1)
        rospy.Subscriber('/fix', NavSatFix, self.gnss_update, queue_size=1)
        rospy.Subscriber('/resolve_navigation_failure_success', Bool, self.resolved_callback, queue_size=1)
        rospy.Subscriber('/explicit_nav_failure', String, self.explicit_fail_callback, queue_size=1)

        self.navigation_monitoring()

    def explicit_fail_callback(self, msg):
        """
        Callback that receives information about explicit navigation failures communicated by the operational state
        machine.

        @param msg: callback message
        """
        rospy.loginfo("CONTINGENCY: detected navigation failure -- EXPLICIT NAV FAIL")
        self.contingency_pub.publish(config.NAVIGATION_FAILURES[2])
        self.active_monitoring = False

    def resolved_callback(self, msg):
        """
        Resolver communication callback - determines whether resolution attempt was successful.
        Aggravates to catastrophe in case of failure.

        @param msg: callback message - True: resolution successful / False: resolution failed
        """
        self.active_monitoring = True
        self.recovery_attempts = 0
        if not msg.data:
            self.interrupt_reason_pub.publish(config.NAV_CATA)
            self.aggravate_pub.publish(config.NAV_CATA)

    def gnss_update(self, nav_sat_fix):
        """
        Callback that receives GNSS data to obtain the robot's position from.

        @param nav_sat_fix: GNSS data
        """
        self.robot_pos = (nav_sat_fix.latitude, nav_sat_fix.longitude)

    def navigation_monitoring(self):
        """
        Monitoring for navigation failures.
        """
        while not rospy.is_shutdown():
            if self.active_monitoring:
                if self.recovery_cnt == 1:
                    self.robot_pos_when_started_recovery = self.robot_pos
                if self.recovery_cnt >= config.RECOVERY_LIMIT:
                    # reached limit of recovery attempts without making progress -> < 1m
                    if distance.distance(self.robot_pos, self.robot_pos_when_started_recovery).km \
                            <= config.RECOVERY_PROGRESS_MIN_DIST_THRESH:

                        rospy.loginfo("CONTINGENCY: detected navigation failure -- SUSTAINED RECOVERY")
                        self.contingency_pub.publish(config.NAVIGATION_FAILURES[0])
                        self.active_monitoring = False
                    else:
                        # made some progress, should reconsider from here
                        rospy.loginfo("detected navigation failure -- SUSTAINED RECOVERY, but made some progress"
                                      + " during recoveries -- continuing recovery")
                        self.robot_info_pub.publish(config.NAVIGATION_FAILURES[1])
                        self.recovery_cnt = 1  # resetting counter
                        self.robot_pos_when_started_recovery = self.robot_pos
            else:
                rospy.loginfo("recovery attempts: %s", self.recovery_attempts)
                if self.recovery_attempts >= config.RECOVERY_LIMIT:
                    self.resolution_failure_pub.publish("")

            rospy.sleep(config.NAV_MON_FREQ)

    def nav_status_callback(self, nav_status):
        """
        Callback that receives status updates from the navigation framework (e.g. move_base_flex).

        @param nav_status: status of the navigation
        """
        if len(nav_status.status_list) > 0:
            # the last element in the list is the latest (most recent)
            self.nav_status = nav_status.status_list[-1].status
            if self.nav_status != self.status_before:
                if self.status_before == GoalStatus.ACTIVE and self.nav_status == GoalStatus.ABORTED:
                    # transition from ACTIVE to ABORTED -> recovery
                    if self.active_monitoring:
                        self.recovery_cnt += 1
                    # currently in resolution mode
                    else:
                        self.recovery_attempts += 1
                elif self.nav_status == GoalStatus.SUCCEEDED:
                    # SUCCESS resets recovery counter
                    self.recovery_cnt = 0
                    self.robot_pos_when_started_recovery = None
            self.status_before = self.nav_status


def node():
    """
    Navigation failure monitoring node.
    """
    rospy.init_node('navigation_monitoring')
    rospy.loginfo("launch navigation monitoring..")
    NavigationMonitoring()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
