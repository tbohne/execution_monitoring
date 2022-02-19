#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64
from execution_monitoring import config
from arox_performance_parameters.msg import arox_battery_params

class PowerManagementMonitoring:

    def __init__(self):
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)
        rospy.Subscriber('/watchdog', String, self.watchdog_callback, queue_size=1)

    def watchdog_callback(self, msg):
        if msg.data == config.CONTINGENCY_MSG:
            rospy.loginfo("battery watchdog module signals contingency -- initiate contigency")
            rospy.loginfo(config.POWER_MANAGEMENT_FAILURE_ONE)
            self.contingency_pub.publish(config.POWER_MANAGEMENT_FAILURE_ONE)
        elif msg.data == config.CATASTROPHE_MSG:
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
