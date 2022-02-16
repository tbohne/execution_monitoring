#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class OperatorCommunication:
    """
    Implements different communication channels with human operator.
    """

    def __init__(self):
        rospy.loginfo("launching robot-human communication module..")
        rospy.Subscriber('/request_help_contingency', String, self.contingency_callback, queue_size=1)
        rospy.Subscriber('/request_help_catastrophe', String, self.catastrophe_callback, queue_size=1)
        rospy.Subscriber('/robot_info', String, self.robot_info_callback, queue_size=1)
        self.op_comm_pub = rospy.Publisher('/operator_communication', String, queue_size=1)

    def contingency_callback(self, msg):
        """
        Needs to do something, but robot still works in principle.
        """
        rospy.loginfo("############ CONTINGENCY CASE ############")
        rospy.loginfo("human intervention required, but robot still works in principle..")
        self.op_comm_pub.publish("operator comm.: human intervention required, but robot still works in principle")
        rospy.loginfo("msg from robot: %s", msg.data)

    def catastrophe_callback(self, msg):
        """
        Needs to do something, robot down.
        """
        rospy.loginfo("############ CATASTROPHE CASE ############")
        rospy.loginfo("human intervention required, robot down..")
        self.op_comm_pub.publish("operator comm.: human intervention required, robot down")
        rospy.loginfo("msg from robot: %s", msg.data)

    def robot_info_callback(self, msg):
        """
        Doesn't require immediate action, but good to know, e.g. memory usage 90%.
        """
        rospy.loginfo("############ INFORMATION ############")
        rospy.loginfo("msg from robot: %s", msg.data)

def node():
    rospy.init_node('operator_communication')
    #rospy.wait_for_message('SMACH_runnning', String)
    OperatorCommunication()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
