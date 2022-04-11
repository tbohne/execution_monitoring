#!/usr/bin/env python
from pathlib import Path

import psutil
import rospy
from std_msgs.msg import String, Bool

from execution_monitoring import util, config


class DataMonitoring:
    """
    Provides monitoring solutions for data management failures.
    """

    def __init__(self):
        self.sim_full_disk = False
        self.scan_cnt_before = 0
        self.mission_name = ""

        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.aggravate_pub = rospy.Publisher('/aggravate', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.logging_pub = rospy.Publisher('/scan_successfully_logged', String, queue_size=1)
        self.interrupt_reason_pub = rospy.Publisher('/interrupt_reason', String, queue_size=1)

        rospy.Subscriber('/scan_action', String, self.data_management_failure_monitoring, queue_size=1)
        rospy.Subscriber('/scan_completed', String, self.after_scan_monitoring, queue_size=1)
        rospy.Subscriber('/mission_name', String, self.mission_name_callback, queue_size=1)
        rospy.Subscriber('/sim_full_disk_failure', String, self.sim_full_disk_callback, queue_size=1)
        rospy.Subscriber('/resolve_data_management_failure_success', Bool, self.resolution_callback, queue_size=1)

    def resolution_callback(self, msg):
        """
        Resolver communication callback - determines whether resolution attempt was successful.
        Aggravates to catastrophe in case of failure.

        @param msg: callback message - True: resolution successful / False: resolution failed
        """
        if not msg.data:
            self.interrupt_reason_pub.publish(config.DATA_MANAGEMENT_CATA)
            self.aggravate_pub.publish(config.DATA_MANAGEMENT_CATA)

    def sim_full_disk_callback(self, msg):
        """
        Initiates full disk failure simulation.

        @param msg: callback message - simulation message
        """
        rospy.loginfo("sim full disk failure..")
        self.sim_full_disk = True

    def after_scan_monitoring(self, msg):
        """
        Performs data management monitoring after completed scan operation.

        @param msg: callback message
        """
        rospy.loginfo("reading file after scan logging..")
        scan_cnt_after = self.count_scan_entries()
        if scan_cnt_after != self.scan_cnt_before + 1:
            rospy.loginfo("data management error.. before: %s, after: %s", self.scan_cnt_before, scan_cnt_after)
            self.contingency_pub.publish(config.DATA_MANAGEMENT_FAILURES[1])
        else:
            rospy.loginfo("data management OK - scan successfully logged..")
            self.logging_pub.publish("")

    def specific_scan_check(self):
        """
        Perform specific scan check - reading scan log file before new scan is logged.
        """
        log_file = Path(config.SCAN_PATH + self.mission_name + config.SCAN_FILE_EXTENSION)
        self.scan_cnt_before = 0
        if log_file.is_file():
            rospy.loginfo("reading file before scan logging..")
            self.scan_cnt_before = self.count_scan_entries()

    def count_scan_entries(self):
        """
        Counts the scan entries in scan log file of the current mission.

        @return: scan entries in the log file
        """
        scan_cnt = 0
        try:
            with open(config.SCAN_PATH + self.mission_name + config.SCAN_FILE_EXTENSION, 'r') as scan_log_file:
                for line in scan_log_file.readlines():
                    # new scan begins
                    if "header" in line:
                        scan_cnt += 1
        except Exception as e:
            rospy.loginfo("unable to read scan log: %s", e)
            return -1
        return scan_cnt

    def drive_capacity_check(self):
        """
        Performs general drive capacity check.
        """
        if self.sim_full_disk:
            self.sim_full_disk = False
            disk_use = psutil.disk_usage(config.FULL_DRIVE)
        else:
            disk_use = psutil.disk_usage(config.MONITOR_DRIVE)

        if disk_use.percent > config.FULL_MEMORY_THRESH:
            rospy.loginfo("full memory - cannot save further scans")
            self.contingency_pub.publish(config.DATA_MANAGEMENT_FAILURES[0])
        elif disk_use.percent > config.ALMOST_FULL_MEMORY_THRESH_TWO:
            rospy.loginfo("scans should be backed up externally soon")
            self.robot_info_pub.publish("scans should be backed up externally soon")
        elif disk_use.percent > config.ALMOST_FULL_MEMORY_THRESH_ONE:
            rospy.loginfo("scans should be backed up externally soon")
            self.robot_info_pub.publish("scans should be backed up externally soon")

        if disk_use.percent > config.ALMOST_FULL_MEMORY_THRESH_ONE:
            used = str(disk_use.total / (1024.0 ** 3))
            rospy.loginfo("drive capacity: %s", used + " GB")
            rospy.loginfo("percentage used: %s", disk_use.percent)
            self.robot_info_pub.publish(
                "drive capacity: " + str(used) + " GB, percentage used: " + str(disk_use.percent))

    def data_management_failure_monitoring(self, msg):
        """
        Initiates data management monitoring after scan action is started.

        @param msg: callback message
        """
        rospy.loginfo("start data management monitoring..")
        self.drive_capacity_check()
        if config.ENABLE_SPECIFIC_LASER_SCAN_CHECK:
            self.specific_scan_check()

    def mission_name_callback(self, mission_name):
        """
        Stores the name of the current LTA mission.

        @param mission_name: name of current LTA mission
        """
        self.mission_name = util.parse_mission_name(mission_name)


def node():
    """
    Data management monitoring node.
    """
    rospy.init_node('data_monitoring')
    rospy.wait_for_message('SMACH_running', String)
    rospy.loginfo("launch data monitoring..")
    DataMonitoring()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
