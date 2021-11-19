#!/usr/bin/env python
import rospy
import actionlib
from execution_monitoring.msg import ScanAction, ScanResult
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class DummyScanner():
    """
    A scan action is expected to cause an incoming scan on the /RIEGL topic, which will then be written to a file to also have some form of data management.
    """

    def __init__(self):
        rospy.loginfo("initializing dummy scanner - waiting for tasks..")
        self.perform_action_pub = rospy.Publisher('/scan_action', String, queue_size=1)
        self.mission_name_sub = rospy.Subscriber('/mission_name', String, self.mission_name_callback, queue_size=1)
        self.server = actionlib.SimpleActionServer('dummy_scanner', ScanAction, execute_cb=self.execute_cb, auto_start=False)
        self.result = ScanResult()
        self.mission_name = ""
        self.server.start()

    def mission_name_callback(self, mission_name):
        self.mission_name = mission_name.data.strip().replace(" ", "_").replace(",", "_").replace("__", "_").replace(":", "_")

    def execute_cb(self, goal):
        # TODO put name of the mission in goal to set it as file name of the scan results
        rospy.loginfo("start scanning procedure..")
        scan = None

        try:
            self.perform_action_pub.publish("action")
            # create a new subscription to the topic, receive one message, then unsubscribe
            scan = rospy.wait_for_message("/RIEGL", LaserScan, timeout=60)
        except rospy.ROSException as e:
            rospy.loginfo("problem retrieving laser scan: %s", e)
        rospy.sleep(3)
        
        if scan:
            rospy.loginfo("recorded scan..")
            rospy.loginfo("scan header: %s", scan.header)
            # TODO: replace absolute path - should be launch parameter
            scan_path = "/home/docker/catkin_ws/src/execution_monitoring/execution_monitoring/scans/"
            if self.mission_name == "":
                self.mission_name = "unknown"
                rospy.loginfo("no mission name published - storing scans in %s", scan_path + self.mission_name + ".txt")
            with open(scan_path + self.mission_name + ".txt", 'a') as out_file:
                out_file.write(str(scan))
                out_file.write("\n############################################\n############################################\n")

        self.result.result = "scanning successfully completed"
        self.server.set_succeeded(self.result)
        rospy.loginfo("scanning completed..")


def node():
    """
    Dummy node to simulate the scanning procedure, i.e. to scan on command.
    """
    rospy.init_node('dummy_scanner')
    rospy.wait_for_message('SMACH_runnning', String)
    server = DummyScanner()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
