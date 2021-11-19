#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from pathlib import Path
from execution_monitoring import util, config

class DataMonitoring:

    def __init__(self):
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)
        self.scan_action_sub = rospy.Subscriber('/scan_action', String, self.data_management_failure_monitoring, queue_size=1)
        self.mission_name_sub = rospy.Subscriber('/mission_name', String, self.mission_name_callback, queue_size=1)
    
    def count_scan_entries(self):
        scan_cnt = 0
        with open(config.SCAN_PATH + self.mission_name + ".txt", 'r') as scan_log_file:
            for l in scan_log_file.readlines():
                # new scan begins
                if "header" in l:
                    scan_cnt += 1
        return scan_cnt

    def data_management_failure_monitoring(self, msg):
        rospy.loginfo("start data management monitoring..")
        log_file = Path(config.SCAN_PATH + self.mission_name + ".txt")
        scan_cnt_before = 0
        if log_file.is_file():
            rospy.loginfo("reading file before scan logging..")
            scan_cnt_before = self.count_scan_entries()
        
        # TODO: should actually be the time limit for scanning
        rospy.sleep(10)

        rospy.loginfo("reading file after scan logging..")
        scan_cnt_after = self.count_scan_entries()

        if scan_cnt_after != scan_cnt_before + 1:
            rospy.loginfo("data management error..")
        else:
            rospy.loginfo("data management OK - scan successfully logged..")

    def mission_name_callback(self, mission_name):
        self.mission_name = util.parse_mission_name(mission_name)


def node():
    rospy.init_node('data_monitoring')
    rospy.wait_for_message('SMACH_runnning', String)
    rospy.loginfo("launch data monitoring..")
    DataMonitoring()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass