#!/usr/bin/env python
import rospy
import psutil
from std_msgs.msg import String
from pathlib import Path
from execution_monitoring import util, config

class DataMonitoring:

    def __init__(self):
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)
        self.scan_action_sub = rospy.Subscriber('/scan_action', String, self.data_management_failure_monitoring, queue_size=1)
        self.mission_name_sub = rospy.Subscriber('/mission_name', String, self.mission_name_callback, queue_size=1)

    def specific_scan_check(self):
        log_file = Path(config.SCAN_PATH + self.mission_name + config.SCAN_FILE_EXTENSION)
        scan_cnt_before = 0
        if log_file.is_file():
            rospy.loginfo("reading file before scan logging..")
            scan_cnt_before = self.count_scan_entries()
        
        rospy.sleep(config.SCAN_TIME_LIMIT)
        rospy.loginfo("reading file after scan logging..")
        scan_cnt_after = self.count_scan_entries()

        if scan_cnt_after != scan_cnt_before + 1:
            rospy.loginfo("data management error..")
            self.contingency_pub.publish(config.DATA_MANAGEMENT_FAILURE_TWO)
        else:
            rospy.loginfo("data management OK - scan successfully logged..")
    
    def count_scan_entries(self):
        scan_cnt = 0
        try:
            with open(config.SCAN_PATH + self.mission_name + config.SCAN_FILE_EXTENSION, 'r') as scan_log_file:
                for l in scan_log_file.readlines():
                    # new scan begins
                    if "header" in l:
                        scan_cnt += 1
        except Exception as e:
            rospy.loginfo("unable to read scan log: %s", e)
            return -1
        return scan_cnt

    def drive_capacity_check(self):
        disk_use = psutil.disk_usage(config.MONITOR_DRIVE)
        if disk_use.percent > 99.9:
            rospy.loginfo("full memory - cannot save further scans")
            self.contingency_pub.publish(config.DATA_MANAGEMENT_FAILURE_ONE)
        elif disk_use.percent > 95:
            rospy.loginfo("scans should be backed up externally soon")
        elif disk_use.percent > 90:
            rospy.loginfo("scans should be backed up externally soon")
        if disk_use.percent > 90:
            rospy.loginfo("drive capacity: %s", str(disk_use.total / (1024.0 ** 3)) + " GB")
            rospy.loginfo("percentage used: %s", disk_use.percent)

    def data_management_failure_monitoring(self, msg):
        rospy.loginfo("start data management monitoring..")
        self.drive_capacity_check()
        if config.ENABLE_SPECIFIC_LASER_SCAN_CHECK:
            self.specific_scan_check()

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
