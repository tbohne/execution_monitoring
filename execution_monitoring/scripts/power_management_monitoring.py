#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from execution_monitoring import config

class PowerManagementMonitoring:

    def __init__(self):
        self.active_catastrophe_monitoring = True
        self.active_contingency_monitoring = True
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)
        rospy.Subscriber('/watchdog', String, self.watchdog_callback, queue_size=1)
        rospy.Subscriber('/fully_charged', String, self.fully_charged_callback, queue_size=1)
        rospy.Subscriber('/catastrophe_launched', String, self.catastrophe_callback, queue_size=1)

    def catastrophe_callback(self, msg):
        self.active_catastrophe_monitoring = False

    def fully_charged_callback(self, msg):
        self.active_catastrophe_monitoring = True
        self.active_contingency_monitoring = True

    def watchdog_callback(self, msg):
        if self.active_contingency_monitoring and msg.data == config.CONTINGENCY_MSG:
            rospy.loginfo("battery watchdog module signals contingency -- initiate contigency")
            rospy.loginfo(config.POWER_MANAGEMENT_FAILURE_ONE)
            self.contingency_pub.publish(config.POWER_MANAGEMENT_FAILURE_ONE)
            self.active_contingency_monitoring = False
        elif self.active_catastrophe_monitoring and msg.data == config.CATASTROPHE_MSG:
            rospy.loginfo("battery watchdog module signals catastrophe -- initiate catastrophe")
            rospy.loginfo(config.POWER_MANAGEMENT_FAILURE_TWO)
            self.catastrophe_pub.publish(config.POWER_MANAGEMENT_FAILURE_TWO)

def node():
    rospy.init_node('power_management_monitoring')
    rospy.loginfo("launch power management monitoring..")
    PowerManagementMonitoring()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
