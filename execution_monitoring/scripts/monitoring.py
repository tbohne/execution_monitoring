#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from execution_monitoring import config

class SensorMonitoring:

    def __init__(self):
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)
        self.scan_action_sub = rospy.Subscriber('/scan_action', String, self.sensor_failure_monitoring, queue_size=1)
        self.latest_scan = None

    def sensor_failure_monitoring(self, msg):
        rospy.loginfo("start sensor monitoring..")
        try:
            # create a new subscription to the topic, receive one message, then unsubscribe
            scan = rospy.wait_for_message("/RIEGL", LaserScan, timeout=60)
        except rospy.ROSException as e:
            rospy.loginfo("SENSOR FAILURE  DETECTED..")
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
