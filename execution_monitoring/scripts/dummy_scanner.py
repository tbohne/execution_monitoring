#!/usr/bin/env python
import rospy
import actionlib
from execution_monitoring.msg import ScanAction, ScanResult
from sensor_msgs.msg import LaserScan

class DummyScanner():

    def __init__(self):
        rospy.loginfo("initializing dummy scanner -  waiting for tasks..")
        self.server = actionlib.SimpleActionServer('dummy_scanner', ScanAction, execute_cb=self.execute_cb, auto_start=False)
        self.result = ScanResult()
        self.server.start()

    def execute_cb(self, goal):
        # TODO put name of the mission in goal to set it as file name of the scan results
        file_name = "test.txt"
        rospy.loginfo(goal)
        rospy.loginfo("start scanning procedure..")

        try:
            # TODO: should actually subscribe to the RIEGL when it's available
            # create a new subscription to the topic, receive one message, then unsubscribe
            scan = rospy.wait_for_message("/scanVelodyne", LaserScan, timeout=60)
        except rospy.ROSException as e:
            rospy.loginfo("problem retrieving laser scan: %s", e)

        rospy.sleep(3)

        if scan:
            rospy.loginfo("recorded scan..")
            rospy.loginfo("scan header: %s", scan.header)
            # TODO: replace absolute path - should be launch parameter
            with open("/home/docker/catkin_ws/src/execution_monitoring/execution_monitoring/scans/" + file_name, 'a') as out_file:
                out_file.write(str(scan.header))
                out_file.write("\n############################################\n############################################\n")

        self.result.result = "scanning successfully completed"
        self.server.set_succeeded(self.result)
        rospy.loginfo("scanning completed..")


def node():
    """
    Dummy node to simulate the scanning procedure, i.e. to scan on command.
    """
    rospy.init_node('dummy_scanner')
    server = DummyScanner()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
