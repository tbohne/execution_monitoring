#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from execution_monitoring import config, util
import hashlib

# TODO: implement with dynamic reconfigure
SCAN_VALUES_LB_PERCENTAGE = 5

class DataManagementMonitoring:

    def __init__(self):
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)
        self.scan_action_sub = rospy.Subscriber('/scan_action', String, self.data_management_failure_monitoring, queue_size=1)
        self.mission_name_sub = rospy.Subscriber('/mission_name', String, self.mission_name_callback, queue_size=1)

    def data_management_failure_monitoring(self, msg):
        rospy.loginfo("start data management monitoring..")
        # read file - count current entries
        # wait a minute or so
        # count entries again - should be one more -> check the latest..

        rospy.loginfor("reading file before scan logging..")
        with open(config.SCAN_PATH + self.mission_name + ".txt", 'r') as scan_log_file:
            for l in scan_log_file.readlines():
                rospy.loginfo(l)

        rospy.sleep(config.SCAN_TIME_LIMIT)

        rospy.loginfor("reading file after scan logging..")
        with open(config.SCAN_PATH + self.mission_name + ".txt", 'r') as scan_log_file:
            for l in scan_log_file.readlines():
                rospy.loginfo(l)


    def mission_name_callback(self, mission_name):
        self.mission_name = util.parse_mission_name(mission_name)


class SensorMonitoring:

    def __init__(self):
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)
        self.scan_action_sub = rospy.Subscriber('/scan_action', String, self.sensor_failure_monitoring, queue_size=1)
        self.previous_scan = None

    def compute_scan_hash(self, scan):
        scan_str = str(scan.header.stamp.secs) + str(scan.header.stamp.nsecs) + scan.header.frame_id + str(scan.angle_min) \
            + str(scan.angle_max) + str(scan.angle_increment) + str(scan.time_increment) + str(scan.scan_time) + str(scan.range_min) \
            + str(scan.range_max) + str(scan.ranges)

        return hashlib.sha224(scan_str.encode('utf-8')).hexdigest()

    def repeated_scan(self, scan):
        new_scan_hash = self.compute_scan_hash(scan)
        prev_scan_hash = self.compute_scan_hash(self.previous_scan)
        rospy.loginfo("new hash: %s", new_scan_hash)
        rospy.loginfo("prev hash: %s", prev_scan_hash)
        return new_scan_hash == prev_scan_hash

    def sensor_failure_monitoring(self, msg):
        rospy.loginfo("start sensor monitoring..")
        scan = None
        try:
            # create a new subscription to the topic, receive one message, then unsubscribe
            scan = rospy.wait_for_message("/RIEGL", LaserScan, timeout=config.SCAN_TIME_LIMIT)
        except rospy.ROSException as e:
            rospy.loginfo("sensor failure detected: %s", config.SENSOR_FAILURE_ONE)
            self.contingency_pub.publish(config.SENSOR_FAILURE_ONE)

        if scan:
            # scan arrived - no total sensor failure, but the remaining sensor failures could still be present
            if len(scan.ranges) == 0:
                rospy.loginfo("sensor failure detected: %s", config.SENSOR_FAILURE_TWO)
                self.contingency_pub.publish(config.SENSOR_FAILURE_TWO)
            else:
                feasible_scans = [s for s in scan.ranges if s != float('inf')]
                if float(len(feasible_scans)) / float(len(scan.ranges)) * 100.0 < SCAN_VALUES_LB_PERCENTAGE:
                    rospy.loginfo("sensor failure detected: %s", config.SENSOR_FAILURE_THREE)
                    self.contingency_pub.publish(config.SENSOR_FAILURE_THREE)
                elif self.previous_scan and self.repeated_scan(scan):
                    rospy.loginfo("sensor failure detected: %s", config.SENSOR_FAILURE_FOUR)
                    self.contingency_pub.publish(config.SENSOR_FAILURE_FOUR)

            self.previous_scan = scan
        else:
            rospy.loginfo("sensor failure detected: %s", config.SENSOR_FAILURE_ONE)
            self.contingency_pub.publish(config.SENSOR_FAILURE_ONE)
    

def node():
    rospy.init_node('monitoring')
    rospy.wait_for_message('SMACH_runnning', String)
    rospy.loginfo("launch monitoring..")
    SensorMonitoring()
    DataManagementMonitoring()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
