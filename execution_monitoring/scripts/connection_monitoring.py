#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
from execution_monitoring.msg import WiFi, Internet
from execution_monitoring import util, config

class ConnectionMonitoring:

    def __init__(self):
        rospy.Subscriber('/wifi_connectivity_info', WiFi, self.wifi_callback, queue_size=1)
        rospy.Subscriber('/internet_connectivity_info', Internet, self.internet_callback, queue_size=1)
        rospy.Subscriber('/resolve_wifi_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_internet_failure_success', Bool, self.resolve_callback, queue_size=1)
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.active_monitoring = True

    def resolve_callback(self, msg):
        self.active_monitoring = True

    def check_wifi_disconnect(self, wifi_info):
        if wifi_info.link_quality == wifi_info.signal_level == wifi_info.bit_rate == 0:
            rospy.loginfo("detected wifi disconnect..")
            self.contingency_pub.publish(config.CONNECTION_FAILURE_FOUR)
            self.active_monitoring = False
            return True
        return False

    def check_internet_disconnect(self, internet_info):
        if internet_info.download == internet_info.upload == 0:
            rospy.loginfo("detected internet disconnect..")
            self.contingency_pub.publish(config.CONNECTION_FAILURE_FIVE)
            self.active_monitoring = False
            return True
        return False

    def check_link_quality(self, link_quality):
        if link_quality < 5:
            rospy.loginfo("detected critically bad wifi link of %s%% - should be fixed..", link_quality)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_ONE)
            self.active_monitoring = False
        elif link_quality < 25:
            rospy.loginfo("detected bad wifi link of %s%%..", link_quality)
            self.robot_info_pub.publish("detected bad wifi link of " + str(link_quality) + "%")
        elif link_quality < 50:
            rospy.loginfo("detected below-average wifi link quality of %s%%..", link_quality)
            self.robot_info_pub.publish("detected below-average wifi link quality of " + str(link_quality) + "%")

    def check_signal_level(self, signal_level):
        if signal_level <= -90:
            rospy.loginfo("detected critically bad wifi signal level of %s dBm - no signal..", signal_level)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_TWO)
            self.active_monitoring = False
        elif signal_level <= -80:
            rospy.loginfo("detected very low wifi signal level of %s dBm", signal_level)
            self.robot_info_pub.publish("detected very low wifi signal level of " + str(signal_level) + "dBm")
        elif signal_level < -75:
            rospy.loginfo("detected low wifi signal level of %s dBm", signal_level)
            self.robot_info_pub.publish("detected low wifi signal level of " + str(signal_level) + "dBm")

    def check_bit_rate(self, bit_rate):
        if bit_rate < 1:
            rospy.loginfo("detected critically bad wifi bit rate of %s Mb/s", bit_rate)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_THREE)
            self.active_monitoring = False
        elif bit_rate < 20:
            rospy.loginfo("detected rather low wifi bit rate of %s Mb/s", bit_rate)
            self.robot_info_pub.publish("detected rather low wifi bit rate of " + str(bit_rate) + "Mb/s")

    def wifi_callback(self, wifi_info):
        if self.active_monitoring:
            rospy.loginfo("receiving new wifi info..")
            rospy.loginfo("link quality: %s%%, signal level: %s dBm, bit rate: %s Mb/s",  "{:4.2f}".format(wifi_info.link_quality),  "{:4.2f}".format(wifi_info.signal_level),  "{:4.2f}".format(wifi_info.bit_rate))
            if not self.check_wifi_disconnect(wifi_info):
                self.check_link_quality(wifi_info.link_quality)
                self.check_signal_level(wifi_info.signal_level)
                self.check_bit_rate(wifi_info.bit_rate)

    def check_download_speed(self, download):
        if download < 1:
            rospy.loginfo("detected critically bad download speed: %s Mb/s", download)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_SIX)
            self.active_monitoring = False
        elif download < 40:
            rospy.loginfo("detected rather low download speed: %s Mb/s", download)
            self.robot_info_pub.publish("detected rather low download speed of " + str(download) + " Mb/s")

    def check_upload_speed(self, upload):
        if upload < 1:
            rospy.loginfo("detected critically bad upload speed: %s Mb/s", upload)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_SEVEN)
            self.active_monitoring = False
        elif upload < 10:
            rospy.loginfo("detected rather low upload speed: %s Mb/s", upload)
            self.robot_info_pub.publish("detected rather low upload speed of " + str(upload) + " Mb/s")

    def internet_callback(self, internet_info):
        if self.active_monitoring:
            rospy.loginfo("receiving new internet connection info..")
            rospy.loginfo("download: %s Mb/s, upload: %s Mb/s", internet_info.download, internet_info.upload)
            if not self.check_internet_disconnect(internet_info):
                self.check_download_speed(internet_info.download)
                self.check_upload_speed(internet_info.upload)

def node():
    rospy.init_node('connection_monitoring')
    rospy.wait_for_message('SMACH_runnning', String)
    rospy.loginfo("launch connection monitoring..")
    ConnectionMonitoring()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
