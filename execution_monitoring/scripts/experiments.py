#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class Experiment:

    def __init__(self):
        self.contingency_cnt = 0
        self.catastrophe_cnt = 0

        rospy.Subscriber("/contingency_preemption", String, self.contingency_callback)
        rospy.Subscriber("/catastrophe_preemption", String, self.catastrophe_callback)

        self.log_info()

    def contingency_callback(self, msg):
        self.contingency_cnt += 1

    def catastrophe_callback(self, msg):
        self.catastrophe_cnt += 1

    def log_info(self):
        while not rospy.is_shutdown():
            rospy.loginfo("contingency cnt: %s", self.contingency_cnt)
            rospy.loginfo("catastrophe cnt: %s", self.catastrophe_cnt)
            rospy.sleep(30)

def node():
    rospy.init_node('experiments')
    Experiment()
    rospy.loginfo("launch experiments node..")
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
