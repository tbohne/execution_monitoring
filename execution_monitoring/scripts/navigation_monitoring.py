#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from execution_monitoring import config, util
from sensor_msgs.msg import NavSatFix
from geopy import distance

class NavigationMonitoring:

    def __init__(self):
        self.mbf_status = None
        self.status_before = None
        self.active_monitoring = True
        self.recovery_cnt = 0
        self.robot_pos = None
        self.robot_pos_when_started_recovery = None
        self.recovery_attempts = 0

        rospy.Subscriber('/move_base_flex/exe_path/status', GoalStatusArray, self.mbf_status_callback, queue_size=1)
        rospy.Subscriber('/fix', NavSatFix, self.gnss_update, queue_size=1)
        rospy.Subscriber('/resolve_navigation_failure_success', Bool, self.resolved_callback, queue_size=1)
        rospy.Subscriber('/explicit_nav_failure', String, self.explicit_fail_callback, queue_size=1)

        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.resolution_failure_pub = rospy.Publisher('/resolution_failure', String, queue_size=1)
        self.navigation_monitoring()

    def explicit_fail_callback(self, msg):
        rospy.loginfo("CONTINGENCY: detected navigation failure -- SUSTAINED RECOVERY")
        self.contingency_pub.publish(config.NAV_FAILURE_THREE)
        self.active_monitoring = False

    def resolved_callback(self, msg):
        self.active_monitoring = True
        self.recovery_attempts  = 0
        if not msg.data:
            self.catastrophe_pub.publish(config.NAV_CATA)

    def gnss_update(self, nav_sat_fix):
        self.robot_pos = (nav_sat_fix.latitude, nav_sat_fix.longitude)

    def navigation_monitoring(self):
        while not rospy.is_shutdown():

            if self.active_monitoring:
                if self.recovery_cnt == 1:
                    self.robot_pos_when_started_recovery = self.robot_pos
                if self.recovery_cnt >= config.RECOVERY_LIMIT:
                    # reached limit of recovery attempts without making progress -> < 1m
                    if distance.distance(self.robot_pos, self.robot_pos_when_started_recovery).km <= 0.001:
                        rospy.loginfo("CONTINGENCY: detected navigation failure -- SUSTAINED RECOVERY")
                        self.contingency_pub.publish(config.NAV_FAILURE_ONE)
                        self.active_monitoring = False
                    else:
                        # made some progress, should reconsider from here on
                        rospy.loginfo("detected navigation failure -- SUSTAINED RECOVERY, but made some progress during recoveries -- continuing recovery")
                        self.robot_info_pub.publish(config.NAV_FAILURE_TWO)
                        self.recovery_cnt = 1
                        self.robot_pos_when_started_recovery = self.robot_pos
            else:
                rospy.loginfo("recovery attempts: %s", self.recovery_attempts)
                if self.recovery_attempts >= config.RECOVERY_LIMIT:
                    self.resolution_failure_pub.publish("")

            # TODO: user-specified frequency
            rospy.sleep(5)

    def mbf_status_callback(self, mbf_status):
        if len(mbf_status.status_list) > 0:
            # the last element in the list is the latest (most recent)
            self.mbf_status = mbf_status.status_list[-1].status
            if self.mbf_status != self.status_before:

                #rospy.loginfo("STATUS: %s", mbf_status.status_list[-1])

                if self.status_before == config.GOAL_STATUS_ACTIVE and self.mbf_status == config.GOAL_STATUS_ABORTED:
                    # transition from ACTIVE to ABORTED -> recovery
                    if self.active_monitoring:
                        self.recovery_cnt += 1
                    # currently in resolution mode
                    else:
                        self.recovery_attempts += 1
                elif self.mbf_status == config.GOAL_STATUS_SUCCEEDED:
                    # SUCCESS resets recovery counter
                    self.recovery_cnt = 0
                    self.robot_pos_when_started_recovery = None
            self.status_before = self.mbf_status


def node():
    rospy.init_node('navigation_monitoring')
    rospy.loginfo("launch navigation monitoring..")
    NavigationMonitoring()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
