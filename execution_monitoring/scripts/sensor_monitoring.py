#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from execution_monitoring import config, util
import hashlib


class SensorMonitoring:

    def __init__(self):
        rospy.Subscriber('/scan_action', String, self.sensor_failure_monitoring, queue_size=1)
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)

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
                if float(len(feasible_scans)) / float(len(scan.ranges)) * 100.0 < config.SCAN_VALUES_LB_PERCENTAGE:
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
    rospy.init_node('sensor_monitoring')
    rospy.wait_for_message('SMACH_runnning', String)
    rospy.loginfo("launch sensor monitoring..")
    SensorMonitoring()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
