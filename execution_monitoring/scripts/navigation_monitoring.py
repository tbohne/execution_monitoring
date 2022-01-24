#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from execution_monitoring import config, util

class NavigationMonitoring:

    def __init__(self):
        rospy.Subscriber('/move_base_flex/exe_path/status', GoalStatusArray, self.mbf_status_callback, queue_size=1)
        self.mbf_status = None
        self.status_before = None
        self.active_monitoring = True
        self.recovery_cnt = 0
        self.navigation_monitoring()

    def navigation_monitoring(self):
        while not rospy.is_shutdown():
            if self.recovery_cnt >= config.RECOVERY_LIMIT:
                rospy.loginfo("DETECTED NAVIGATION FAILURE -- SUSTAINED RECOVERY")
            rospy.sleep(5)

    def mbf_status_callback(self, mbf_status):
        if self.active_monitoring and len(mbf_status.status_list) > 0:
            # the last element in the list is the latest (most recent)
            self.mbf_status = mbf_status.status_list[-1].status
            if self.mbf_status != self.status_before:
                if self.status_before == config.GOAL_STATUS_ACTIVE and self.mbf_status == config.GOAL_STATUS_ABORTED:
                    self.recovery_cnt += 1
                elif self.mbf_status == config.GOAL_STATUS_SUCCEEDED:
                    self.recovery_cnt = 0    
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
