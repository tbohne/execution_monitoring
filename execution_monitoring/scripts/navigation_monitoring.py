#!/usr/bin/env python
import rospy
from std_msgs.msg import String
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

        rospy.Subscriber('/move_base_flex/exe_path/status', GoalStatusArray, self.mbf_status_callback, queue_size=1)
        rospy.Subscriber('/fix', NavSatFix, self.gnss_update, queue_size=1)

        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.navigation_monitoring()

    def gnss_update(self, nav_sat_fix):
        self.robot_pos = (nav_sat_fix.latitude, nav_sat_fix.longitude)

    def navigation_monitoring(self):
        while not rospy.is_shutdown():
            if self.recovery_cnt == 1:
                self.robot_pos_when_started_recovery = self.robot_pos

            if self.recovery_cnt >= config.RECOVERY_LIMIT:
                # reached limit of recovery attempts without making progress -> < 1m
                if distance.distance(self.robot_pos, self.robot_pos_when_started_recovery).km <= 0.001:
                    rospy.loginfo("CONTINGENCY: detected navigation failure -- SUSTAINED RECOVERY")
                    self.contingency_pub.publish(config.NAV_FAILURE_ONE)
                else:
                    # made some progress, should reconsider from here on
                    rospy.loginfo("detected navigation failure -- SUSTAINED RECOVERY, but made some progress during recoveries -- continuing recovery")
                    self.robot_info_pub.publish(config.NAV_FAILURE_TWO)
                    self.recovery_cnt = 1
                    self.robot_pos_when_started_recovery = self.robot_pos
            rospy.sleep(5)

    def mbf_status_callback(self, mbf_status):
        if self.active_monitoring and len(mbf_status.status_list) > 0:
            # the last element in the list is the latest (most recent)
            self.mbf_status = mbf_status.status_list[-1].status
            if self.mbf_status != self.status_before:
                if self.status_before == config.GOAL_STATUS_ACTIVE and self.mbf_status == config.GOAL_STATUS_ABORTED:
                    # transition from ACTIVE to ABORTED -> recovery
                    self.recovery_cnt += 1
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
