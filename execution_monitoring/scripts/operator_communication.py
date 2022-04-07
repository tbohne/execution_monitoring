#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class OperatorCommunication:
    """
    Implements different communication channels with human operator.
    """

    def __init__(self):
        rospy.loginfo("launching robot-human communication module..")
        rospy.Subscriber('/request_help', String, self.request_help_callback, queue_size=1)
        rospy.Subscriber('/robot_info', String, self.robot_info_callback, queue_size=1)
        self.op_comm_pub = rospy.Publisher('/operator_communication', String, queue_size=1)

    def request_help_callback(self, msg):
        """
        Robot requests help of human operator - not able to solve the problem itself.
        """
        rospy.loginfo("############ CATASTROPHE CASE ############")
        rospy.loginfo("human intervention required")
        self.op_comm_pub.publish("operator comm.: catastrophe -- human intervention required")
        rospy.loginfo("msg from robot: %s", msg.data)

    def robot_info_callback(self, msg):
        """
        Doesn't require immediate action, but good to know, e.g. memory usage 90%.
        """
        rospy.loginfo("############ INFORMATION ############")
        rospy.loginfo("msg from robot: %s", msg.data)

def node():
    rospy.init_node('operator_communication')
    #rospy.wait_for_message('SMACH_running', String)
    OperatorCommunication()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
