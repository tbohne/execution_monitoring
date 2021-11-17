#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from execution_monitoring import config
import hashlib

# TODO: implement with dynamic reconfigure
SCAN_VALUES_LB_PERCENTAGE = 5

class SensorMonitoring:

    def __init__(self):
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)
        self.scan_action_sub = rospy.Subscriber('/scan_action', String, self.sensor_failure_monitoring, queue_size=1)
        self.previous_scan = None

    def repeated_scan(self, scan):
        new_scan_str = str(scan.header.stamp.secs) + str(scan.header.stamp.nsecs) + scan.header.frame_id
        prev_scan_str = str(self.previous_scan.header.stamp.secs) + str(self.previous_scan.header.stamp.nsecs) + self.previous_scan.header.frame_id
        new_hash = hashlib.sha224(new_scan_str.encode('utf-8')).hexdigest()
        prev_hash = hashlib.sha224(prev_scan_str.encode('utf-8')).hexdigest()
        rospy.loginfo("new hash: %s", new_hash)
        rospy.loginfo("prev hash: %s", prev_hash)
        return new_hash == prev_hash

    def sensor_failure_monitoring(self, msg):
        rospy.loginfo("start sensor monitoring..")
        scan = None
        try:
            # create a new subscription to the topic, receive one message, then unsubscribe
            scan = rospy.wait_for_message("/RIEGL", LaserScan, timeout=60)
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

                rospy.loginfo("monitoring: curr scan: %s", scan.header)
                if self.previous_scan:
                    rospy.loginfo("monitoring: prev scan: %s", self.previous_scan.header)

            self.previous_scan = scan
        else:
            rospy.loginfo("sensor failure detected: %s", config.SENSOR_FAILURE_ONE)
            self.contingency_pub.publish(config.SENSOR_FAILURE_ONE)
    

def node():
    rospy.init_node('monitoring')
    rospy.wait_for_message('SMACH_runnning', String)
    rospy.loginfo("launch sensor monitoring..")
    monitor = SensorMonitoring()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
