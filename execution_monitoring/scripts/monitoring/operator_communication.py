#!/usr/bin/env python
import rospy
from std_msgs.msg import String


class OperatorCommunication:
    """
    Class for communication with a human operator, i.e., notifying a human operator that a problem has occurred that
    requires human intervention, or providing useful information about the state of the robot or mission.
    """

    def __init__(self):
        rospy.loginfo("launching robot-human communication module..")
        rospy.Subscriber('/request_help', String, self.request_help_callback, queue_size=1)
        rospy.Subscriber('/robot_info', String, self.robot_info_callback, queue_size=1)
        self.op_comm_pub = rospy.Publisher('/operator_communication', String, queue_size=1)

    def request_help_callback(self, msg):
        """
        Robot requests help of human operator - not able to solve a problem itself.

        @param msg: callback message - help request of the robot
        """
        rospy.loginfo("############ CATASTROPHE CASE ############")
        rospy.loginfo("human intervention required")
        self.op_comm_pub.publish("operator comm.: catastrophe -- human intervention required")
        rospy.loginfo("msg from robot: %s", msg.data)

    @staticmethod
    def robot_info_callback(msg):
        """
        Robot communicates minor problems or tasks that do not require immediate action, but are good to know and
        tackle soon, e.g., a memory usage of 90%. Yet it is also used for simple status updates on the robot or its
        environment, such as weather conditions.

        @param msg: callback message - status information provided by the robot
        """
        rospy.loginfo("############ INFORMATION ############")
        rospy.loginfo("msg from robot: %s", msg.data)


def node():
    """
    Robot-human communication node.
    """
    rospy.init_node('operator_communication')
    OperatorCommunication()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
