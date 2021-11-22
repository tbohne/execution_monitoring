#!/usr/bin/env python
import rospy
import subprocess
import re
import datetime
from execution_monitoring import config

def now():
    return datetime.datetime.now().strftime("%Y-%m-%d %H:%M")

def monitor_wifi():
    while not rospy.is_shutdown():
        p = subprocess.Popen(["iwconfig", config.WIFI_INTERFACE], stdout=subprocess.PIPE)
        out, err = p.communicate()
        m = re.search(r"Link.*.dBm", out)
        rospy.loginfo(now() + " --- " + m.group(0))
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
