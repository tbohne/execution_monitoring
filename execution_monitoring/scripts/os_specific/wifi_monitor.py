#!/usr/bin/env python
import rospy
import subprocess
import re

def monitor_wifi():
    p = subprocess.Popen(["iwconfig", "wlx3c1e045678a2"], stdout=subprocess.PIPE)
    out, err = p.communicate()
    m = re.search(r"Link.*.dBm", out)
    print(m.group(0))

def node():
    rospy.init_node('wifi_monitor')
    monitor_wifi()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
