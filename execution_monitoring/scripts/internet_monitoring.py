#!/usr/bin/env python
import rospy
import speedtest
import datetime
from execution_monitoring.msg import Internet


class InternetConnectionMonitor:

    def __init__(self):
        self.internet_info_pub = rospy.Publisher('/internet_connectivity_info', Internet, queue_size=1)
        self.test = speedtest.Speedtest()
        self.monitor_internet_connection()

    def monitor_internet_connection(self):
        
        while not rospy.is_shutdown():
            time_now = datetime.datetime.now().strftime("%H:%M:%S")
            download = round((round(self.test.download()) / 2 ** 20), 2)
            upload = round((round(self.test.upload()) / 2 ** 20), 2)
            rospy.loginfo("time: %s, download: %s, upload: %s", time_now, download, upload)

            internet_msg = Internet()
            internet_msg.download = download
            internet_msg.upload = upload

            self.internet_info_pub.publish(internet_msg)
            rospy.sleep(10)

def node():
    rospy.init_node('internet_monitor')
    InternetConnectionMonitor()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
