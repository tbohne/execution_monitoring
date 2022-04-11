#!/usr/bin/env python
import datetime
import re
import subprocess

import rospy
from std_msgs.msg import String

from execution_monitoring import config
from execution_monitoring.msg import WiFi


class WiFiMonitor:
    """
    Operating system (Ubuntu) specific monitoring node that transfers information about the WiFi connection to the ROS
    system. It starts the Unix tool `iwconfig` as a subprocess for a WiFi interface identifier that can be configured
    by the user. Subsequently, the relevant information, i.e., link quality, signal level and bit rate, is parsed from
    the `iwconfig` output for the respective interface and a WiFi.msg is created based on this. This message is then
    published under the `/wifi_connectivity_info` topic, which is subscribed by the high-level monitoring node.
    --------------------------------------------------------------------------------------------------------------------
    Monitored aspects:
    - link_quality (%): measure of how good the link is (including signal strength, speed, packet loss, retries, etc.)
    - signal_level (dBm): received signal strength indication (RSSI)
        - dBm (decibel-milliwatts) - commonly used metric to measure wifi signal strength or power
        - -50 dBm excellent, -60 dBm very good, -70 dBm good, -80 dBm low, -90 dBm very low, -100 dBm no signal
    - bit_rate (Mb/s): speed at which bits are transmitted over the medium
    --------------------------------------------------------------------------------------------------------------------
    If the WiFi is not connected at all, the WiFi connection monitoring node is expected to set all three values to 0.
    """

    def __init__(self):
        self.simulate_bad_wifi_link = False
        self.simulate_bad_wifi_signal = False
        self.simulate_bad_wifi_bit_rate = False
        self.simulate_wifi_disconnect = False
        rospy.Subscriber("/toggle_simulated_bad_wifi_link", String, self.bad_wifi_link_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_bad_wifi_signal", String, self.bad_wifi_signal_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_bad_wifi_bit_rate", String, self.bad_wifi_bit_rate_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_wifi_disconnect", String, self.wifi_disconnect_callback, queue_size=1)
        self.wifi_info_pub = rospy.Publisher('/wifi_connectivity_info', WiFi, queue_size=1)
        self.monitor_wifi()

    def bad_wifi_link_callback(self, msg):
        """
        Toggles bad WiFi link simulation.

        :param msg: callback message
        """
        self.simulate_bad_wifi_link = not self.simulate_bad_wifi_link

    def bad_wifi_signal_callback(self, msg):
        """
        Toggles bad WiFi signal simulation.

        :param msg: callback message
        """
        self.simulate_bad_wifi_signal = not self.simulate_bad_wifi_signal

    def bad_wifi_bit_rate_callback(self, msg):
        """
        Toggles bad WiFi bit rate simulation.

        :param msg: callback message
        """
        self.simulate_bad_wifi_bit_rate = not self.simulate_bad_wifi_bit_rate

    def wifi_disconnect_callback(self, msg):
        """
        Toggles WiFi disconnect simulation.

        :param msg: callback message
        """
        self.simulate_wifi_disconnect = not self.simulate_wifi_disconnect

    @staticmethod
    def now():
        """
        Returns the current date and time.

        :return: current date and time
        """
        return datetime.datetime.now().strftime("%Y-%m-%d %H:%M")

    @staticmethod
    def parse_relevant_parameters(iwconfig_input):
        """
        Parses the relevant parameters (link quality, signal level, bit rate) from the `iwconfig` input.

        :param iwconfig_input: output of the Unix tool `iwconfig` for the particular WiFi interface
        :return: (link quality, signal level, bit rate)
        """
        if "Link Quality" in iwconfig_input:
            first = re.search(r"Link Quality.*", iwconfig_input).group(0).strip().split("  ")
            sec = re.search(r"Bit Rate.*", iwconfig_input).group(0).strip().split("  ")
            link_quality, link_quality_UB = first[0].split("=")[1].split("/")
            link_quality_percentage = float(link_quality) / float(link_quality_UB) * 100
            signal_level = float(first[1].split("=")[1].split(" ")[0])
            bit_rate = float(sec[0].split("=")[1].split(" ")[0])
            return link_quality_percentage, signal_level, bit_rate
        else:
            return 0, 0, 0  # WiFi not connected

    def monitor_wifi(self):
        """
        Monitors the connection status of the specified WiFi interface.
        """
        while not rospy.is_shutdown():
            p = subprocess.Popen(["iwconfig", config.WIFI_INTERFACE], stdout=subprocess.PIPE)
            out, err = p.communicate()
            link_quality, signal_level, bit_rate = self.parse_relevant_parameters(out)

            if config.VERBOSE_LOGGING:
                rospy.loginfo("%s --- link quality: %s%%, signal level: %s dBm, bit rate: %s Mb/s", self.now(),
                              "{:4.2f}".format(link_quality), "{:4.2f}".format(signal_level),
                              "{:4.2f}".format(bit_rate))

            wifi_msg = WiFi()
            wifi_msg.link_quality = link_quality
            wifi_msg.signal_level = signal_level
            wifi_msg.bit_rate = bit_rate

            if self.simulate_wifi_disconnect:
                wifi_msg.link_quality = wifi_msg.signal_level = wifi_msg.bit_rate = config.WIFI_DISCONNECT
                self.simulate_wifi_disconnect = False
            if self.simulate_bad_wifi_link:
                wifi_msg.link_quality = config.BAD_WIFI_LINK_QUALITY
                self.simulate_bad_wifi_link = False
            if self.simulate_bad_wifi_signal:
                wifi_msg.signal_level = config.BAD_WIFI_SIGNAL_LEVEL
                self.simulate_bad_wifi_signal = False
            if self.simulate_bad_wifi_bit_rate:
                wifi_msg.bit_rate = config.BAD_WIFI_BIT_RATE
                self.simulate_bad_wifi_bit_rate = False

            self.wifi_info_pub.publish(wifi_msg)
            rospy.sleep(config.WIFI_MONITORING_FREQ)


def node():
    """
    Operating system (Ubuntu) specific monitoring node that transfers information about the WiFi connection to the ROS
    system.
    """
    rospy.init_node('wifi_monitor')
    WiFiMonitor()
    rospy.loginfo("launch wifi monitoring..")
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
