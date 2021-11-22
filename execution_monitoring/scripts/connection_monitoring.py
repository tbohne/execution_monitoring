#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from execution_monitoring.msg import WiFi
from execution_monitoring import util, config

class ConnectionMonitoring:

    def __init__(self):
        self.wifi_info_sub = rospy.Subscriber('/wifi_connectivity_info', WiFi, self.wifi_callback, queue_size=1)

    def wifi_callback(self, wifi_info):
        rospy.loginfo("receiving new wifi info..")
        rospy.loginfo(wifi_info)

def node():
    rospy.init_node('connection_monitoring')
    # rospy.wait_for_message('SMACH_runnning', String)
    rospy.loginfo("launch connection monitoring..")
    ConnectionMonitoring()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
