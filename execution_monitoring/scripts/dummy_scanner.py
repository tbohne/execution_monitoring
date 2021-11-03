#!/usr/bin/env python
import rospy
import actionlib
from execution_monitoring.msg import ScanAction

class DummyScanner():

    def __init__(self):
        self.server = actionlib.SimpleActionServer('dummy_scanner', ScanAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()
        
    def execute_cb(self, goal):
        rospy.loginfo("goal: %s", goal)
        rospy.loginfo("start scanning procedure..")
        rospy.sleep(10)
        self.server.set_succeeded(True)


def node():
    """
    Dummy node to simulate the scanning procedure, i.e. to scan on command.
    """
    rospy.init_node('dummy_scanner')

    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
