#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from execution_monitoring import config

class ChargingFailureMonitoring:

    def __init__(self):
        rospy.Subscriber('/explicit_charging_failure', String, self.explicit_failure_callback, queue_size=1)

        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)

    def explicit_failure_callback(self, msg):
        rospy.loginfo("explicit charging fail communicated by operation smach..")
        self.contingency_pub.publish(config.CHARGING_FAILURE_ONE)

def node():
    rospy.init_node('charging_failure_monitoring')
    rospy.loginfo("launch charging failure monitoring..")
    ChargingFailureMonitoring()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
