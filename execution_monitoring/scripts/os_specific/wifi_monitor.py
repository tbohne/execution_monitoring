#!/usr/bin/env python
import rospy
import subprocess
import re
import datetime
from execution_monitoring import config

def now():
    return datetime.datetime.now().strftime("%Y-%m-%d %H:%M")

def parse_relevant_parameters(iwconfig_input):
    first = re.search(r"Link Quality.*", iwconfig_input).group(0).strip().split("  ")
    sec = re.search(r"Bit Rate.*", iwconfig_input).group(0).strip().split("  ")
    link_quality, link_quality_UB = first[0].split("=")[1].split("/")
    link_quality_percentage = round(float(link_quality) / float(link_quality_UB) * 100, 2)
    signal_level = int(first[1].split("=")[1].split(" ")[0])
    bit_rate = float(sec[0].split("=")[1].split(" ")[0])
    return link_quality_percentage, signal_level, bit_rate

def monitor_wifi():

    while not rospy.is_shutdown():
        p = subprocess.Popen(["iwconfig", config.WIFI_INTERFACE], stdout=subprocess.PIPE)
        out, err = p.communicate()
        link_quality, signal_level, bit_rate = parse_relevant_parameters(out)
        rospy.loginfo("%s --- link quality: %s%%, signal level: %s dBm, bit rate: %s Mb/s", now(), link_quality, signal_level, bit_rate)
        rospy.sleep(10)

def node():
    rospy.init_node('wifi_monitor')
    monitor_wifi()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
