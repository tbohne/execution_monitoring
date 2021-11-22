#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from execution_monitoring.msg import WiFi
from execution_monitoring import util, config

class ConnectionMonitoring:

    def __init__(self):
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)
        self.wifi_info_sub = rospy.Subscriber('/wifi_connectivity_info', WiFi, self.wifi_callback, queue_size=1)

    def check_disconnect(self, wifi_info):
        if wifi_info.link_quality == wifi_info.signal_level == wifi_info.bit_rate == 0:
            rospy.loginfo("detected wifi disconnect..")
            self.contingency_pub.publish(config.CONNECTION_FAILURE_FOUR)
            return True
        return False

    def check_link_quality(self, link_quality):
        if link_quality < 5:
            rospy.loginfo("detected critically bad wifi link of %s%% - should be fixed..", link_quality)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_ONE)
        elif link_quality < 25:
            rospy.loginfo("detected bad wifi link of %s%%..", link_quality)
        elif link_quality < 50:
            rospy.loginfo("detected below-average wifi link quality of %s%%..", link_quality)

    def check_signal_level(self, signal_level):
        if signal_level < -95:
            rospy.loginfo("detected critically bad wifi signal level of %s dBm - no signal..", signal_level)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_TWO)
        elif signal_level < -85:
            rospy.loginfo("detected very low wifi signal level of %s dBm", signal_level)
        elif signal_level < -75:
            rospy.loginfo("detected low wifi signal level of %s dBm", signal_level)

    def check_bit_rate(self, bit_rate):
        if bit_rate < 1:
            rospy.loginfo("detected critically bad wifi bit rate of %s Mb/s", bit_rate)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_THREE)
        elif bit_rate < 20:
            rospy.loginfo("detected rather low wifi bit rate of %s Mb/s", bit_rate)

    def wifi_callback(self, wifi_info):
        rospy.loginfo("receiving new wifi info..")
        rospy.loginfo("link quality: %s%%, signal level: %s dBm, bit rate: %s Mb/s",  "{:4.2f}".format(wifi_info.link_quality),  "{:4.2f}".format(wifi_info.signal_level),  "{:4.2f}".format(wifi_info.bit_rate))
        if not self.check_disconnect(wifi_info):
            self.check_link_quality(wifi_info.link_quality)
            self.check_signal_level(wifi_info.signal_level)
            self.check_bit_rate(wifi_info.bit_rate)

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
