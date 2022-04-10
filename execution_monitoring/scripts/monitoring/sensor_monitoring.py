#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String, Bool
from execution_monitoring import config, util
import hashlib


class SensorMonitoring:
    """
    Provides monitoring solutions for sensor failures.
    """

    def __init__(self):
        self.previous_scan = None
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.aggravate_pub = rospy.Publisher('/aggravate', String, queue_size=1)
        self.interrupt_reason_pub = rospy.Publisher('/interrupt_reason', String, queue_size=1)
        rospy.Subscriber('/scan_action', String, self.sensor_failure_monitoring, queue_size=1)
        rospy.Subscriber('/resolve_sensor_failure_success', Bool, self.resolution_callback, queue_size=1)

    def resolution_callback(self, msg):
        """
        Resolver communication callback - determines whether resolution attempt was successful.
        Aggravates to catastrophe in case of failure.

        @param msg: callback message - True: resolution successful / False: resolution failed
        """
        if not msg.data:
            self.interrupt_reason_pub.publish(config.SENSOR_CATA)
            self.aggravate_pub.publish(config.SENSOR_CATA)

    @staticmethod
    def compute_scan_hash(scan):
        """
        Computes a hash value that can be used to identify the specified scan.

        @param scan: scan to compute hash for
        @return: computed hash
        """
        scan_str = str(scan.header.stamp.secs) + str(scan.header.stamp.nsecs) + scan.header.frame_id \
                   + str(scan.angle_min) + str(scan.angle_max) + str(scan.angle_increment) + str(scan.time_increment) \
                   + str(scan.scan_time) + str(scan.range_min) + str(scan.range_max) + str(scan.ranges)
        return hashlib.sha224(scan_str.encode('utf-8')).hexdigest()

    def repeated_scan(self, scan):
        """
        Checks whether the specified scan has been recorded before.

        @param scan: scan to be checked
        """
        new_scan_hash = self.compute_scan_hash(scan)
        prev_scan_hash = self.compute_scan_hash(self.previous_scan)
        rospy.loginfo("new hash: %s", new_scan_hash)
        rospy.loginfo("prev hash: %s", prev_scan_hash)
        return new_scan_hash == prev_scan_hash

    def sensor_failure_monitoring(self, msg):
        """
        Performs monitoring for sensor failures, initiated by scan action.

        @param msg: callback message
        """
        rospy.loginfo("start sensor monitoring..")
        scan = None
        try:
            # create a new subscription to the topic, receive one message, then unsubscribe
            scan = rospy.wait_for_message(config.SCAN_TOPIC, LaserScan, timeout=config.SCAN_TIME_LIMIT)
        except rospy.ROSException as e:
            rospy.loginfo("error: %s", e)
            rospy.loginfo("sensor failure detected: %s", config.SENSOR_FAILURES[0])
            self.contingency_pub.publish(config.SENSOR_FAILURES[0])

        if scan:
            # scan arrived - no total sensor failure, but the remaining sensor failures could still be present
            if len(scan.ranges) == 0:
                rospy.loginfo("sensor failure detected: %s", config.SENSOR_FAILURES[1])
                self.contingency_pub.publish(config.SENSOR_FAILURES[1])
            else:
                feasible_scans = [s for s in scan.ranges if s != float('inf')]
                if float(len(feasible_scans)) / float(len(scan.ranges)) * 100.0 < config.SCAN_VALUES_LB_PERCENTAGE:
                    rospy.loginfo("sensor failure detected: %s", config.SENSOR_FAILURES[2])
                    self.contingency_pub.publish(config.SENSOR_FAILURES[2])
                elif self.previous_scan and self.repeated_scan(scan):
                    rospy.loginfo("sensor failure detected: %s", config.SENSOR_FAILURES[3])
                    self.contingency_pub.publish(config.SENSOR_FAILURES[3])

            self.previous_scan = scan


def node():
    """
    Sensor failure monitoring node.
    """
    rospy.init_node('sensor_monitoring')
    rospy.wait_for_message('SMACH_running', String)
    rospy.loginfo("launch sensor monitoring..")
    SensorMonitoring()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
