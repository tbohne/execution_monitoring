#!/usr/bin/env python
import rospy
import speedtest
import datetime
from execution_monitoring.msg import Internet
from std_msgs.msg import String
from execution_monitoring import config


class InternetConnectionMonitor:

    def __init__(self):
        self.simulate_bad_download = False
        self.simulate_bad_upload = False
        rospy.Subscriber('/re_init_internet_monitoring', String, self.re_init, queue_size=1)
        rospy.Subscriber("/toggle_simulated_bad_download", String, self.toggle_bad_download_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_bad_upload", String, self.toggle_bad_upload_callback, queue_size=1)
        self.internet_info_pub = rospy.Publisher('/internet_connectivity_info', Internet, queue_size=1)
        self.connect_to_speedtest()

    def connect_to_speedtest(self):
        try:
            self.test = speedtest.Speedtest()
            self.monitor_internet_connection()
        except Exception as e:
            rospy.loginfo("connection to speedtest API not possible: %s", e)
            # necessary to wait for publishers / subscribers to be ready
            rospy.sleep(3)
            self.disconnect()

    def toggle_bad_download_callback(self, msg):
        self.simulate_bad_download = not self.simulate_bad_download

    def toggle_bad_upload_callback(self, msg):
        self.simulate_bad_upload = not self.simulate_bad_upload

    def re_init(self, msg):
        rospy.loginfo("reinitializing internet monitoring node..%s", msg.data)
        self.connect_to_speedtest()

    def generate_msg(self, down, up):
        internet_msg = Internet()
        internet_msg.download = down
        internet_msg.upload = up
        if self.simulate_bad_download:
            internet_msg.download = config.BAD_DOWNLOAD
            self.simulate_bad_download = False
        if self.simulate_bad_upload:
            internet_msg.upload = config.BAD_UPLOAD
            self.simulate_bad_upload = False
        return internet_msg
    
    def disconnect(self):
        rospy.loginfo("internet disconnected..")
        self.internet_info_pub.publish(self.generate_msg(0, 0))

    def monitor_internet_connection(self):
        
        while not rospy.is_shutdown():
            time_now = datetime.datetime.now().strftime("%H:%M:%S")
            download = round((round(self.test.download()) / 2 ** 20), 2)
            upload = round((round(self.test.upload()) / 2 ** 20), 2)
            rospy.loginfo("time: %s, download: %s Mb/s, upload: %s Mb/s", time_now, download, upload)
            self.internet_info_pub.publish(self.generate_msg(download, upload))
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
