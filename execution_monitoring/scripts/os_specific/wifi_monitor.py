#!/usr/bin/env python
import rospy
import subprocess
import re
import datetime
from execution_monitoring.msg import WiFi
from execution_monitoring import config
from std_msgs.msg import String

class WiFiMonitor:
    """
    Operating system specific node that transfers information about the wifi connection into the ROS world.
    Expected format and information: WiFi.msg
    
        - link_quality (%): measure of how good the link is (including signal strength, speed, packet loss, retries, etc.)
        - signal_level (dBm): received signal strength indication (RSSI)
            - dBm (decibel-milliwatts) - commonly used metric to measure wifi signal strength or power
            - [https://www.netspotapp.com/what-is-rssi-level.html]
            - -50 dBm excellent
            - -60 dBm very good
            - -70 dBm good
            - -80 dBm low
            - -90 dBm very low
            - -100 dBm no signal
        - bit_rate (Mb/s): speed at which bits are transmitted over the medium

    If the wifi is not connected at all, such an OS-specific wifi connection monitoring node is expected to set all three values to 0.
    """

    def __init__(self):
        self.simulate_bad_wifi_link = False
        self.simulate_bad_wifi_signal = False
        self.simulate_bad_wifi_bit_rate = False
        self.simulate_wifi_disconnect = False
        rospy.Subscriber("/toggle_simulated_bad_wifi_link", String, self.toggle_bad_wifi_link_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_bad_wifi_signal", String, self.toggle_bad_wifi_signal_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_bad_wifi_bit_rate", String, self.toggle_bad_wifi_bit_rate_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_wifi_disconnect", String, self.toggle_wifi_disconnect_callback, queue_size=1)
        self.wifi_info_pub = rospy.Publisher('/wifi_connectivity_info', WiFi, queue_size=1)
        self.monitor_wifi()

    def toggle_bad_wifi_link_callback(self, msg):
        self.simulate_bad_wifi_link = not self.simulate_bad_wifi_link

    def toggle_bad_wifi_signal_callback(self, msg):
        self.simulate_bad_wifi_signal = not self.simulate_bad_wifi_signal

    def toggle_bad_wifi_bit_rate_callback(self, msg):
        self.simulate_bad_wifi_bit_rate = not self.simulate_bad_wifi_bit_rate

    def toggle_wifi_disconnect_callback(self, msg):
        self.simulate_wifi_disconnect = not self.simulate_wifi_disconnect

    def now(self):
        return datetime.datetime.now().strftime("%Y-%m-%d %H:%M")

    def parse_relevant_parameters(self, iwconfig_input):
        if "Link Quality" in iwconfig_input:
            first = re.search(r"Link Quality.*", iwconfig_input).group(0).strip().split("  ")
            sec = re.search(r"Bit Rate.*", iwconfig_input).group(0).strip().split("  ")
            link_quality, link_quality_UB = first[0].split("=")[1].split("/")
            link_quality_percentage = float(link_quality) / float(link_quality_UB) * 100
            signal_level = float(first[1].split("=")[1].split(" ")[0])
            bit_rate = float(sec[0].split("=")[1].split(" ")[0])
            return link_quality_percentage, signal_level, bit_rate
        else:
            # wifi not connected
            return 0, 0, 0

    def monitor_wifi(self):
        while not rospy.is_shutdown():
            p = subprocess.Popen(["iwconfig", config.WIFI_INTERFACE], stdout=subprocess.PIPE)
            out, err = p.communicate()
            link_quality, signal_level, bit_rate = self.parse_relevant_parameters(out)
            # rospy.loginfo("%s --- link quality: %s%%, signal level: %s dBm, bit rate: %s Mb/s", self.now(), "{:4.2f}".format(link_quality), "{:4.2f}".format(signal_level), "{:4.2f}".format(bit_rate))

            wifi_msg = WiFi()
            wifi_msg.link_quality = link_quality
            wifi_msg.signal_level = signal_level
            wifi_msg.bit_rate = bit_rate

            if self.simulate_wifi_disconnect:
                wifi_msg.link_quality = wifi_msg.signal_level = wifi_msg.bit_rate = config.WIFI_DISCONNECT
            if self.simulate_bad_wifi_link:
                wifi_msg.link_quality = config.BAD_WIFI_LINK_QUALITY
            if self.simulate_bad_wifi_signal:
                wifi_msg.signal_level = config.BAD_WIFI_SIGNAL_LEVEL
            if self.simulate_bad_wifi_bit_rate:
                wifi_msg.bit_rate = config.BAD_WIFI_BIT_RATE

            self.wifi_info_pub.publish(wifi_msg)
            rospy.sleep(10)

def node():
    rospy.init_node('wifi_monitor')
    WiFiMonitor()
    rospy.loginfo("launch wifi monitoring..")
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
