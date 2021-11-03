#!/usr/bin/env python
import rospy
import actionlib
from execution_monitoring.msg import ScanAction, ScanResult

class DummyScanner():

    def __init__(self):
        rospy.loginfo("initializing dummy scanner -  waiting for tasks..")
        self.server = actionlib.SimpleActionServer('dummy_scanner', ScanAction, execute_cb=self.execute_cb, auto_start=False)
        self.result = ScanResult()
        self.server.start()
        
    def execute_cb(self, goal):
        rospy.loginfo(goal)
        rospy.loginfo("start scanning procedure..")
        rospy.sleep(10)
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
