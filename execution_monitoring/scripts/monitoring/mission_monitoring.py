#!/usr/bin/env python
from datetime import datetime

import rospy
from arox_performance_parameters.msg import arox_operational_param
from std_msgs.msg import String

from execution_monitoring import config


class MissionMonitor:
    """
    Simple mission monitoring.

    For evaluation purposes, let a failed mission be defined as a mission that exceeds a user-configurable timeout
    without publishing a message to "/arox/ongoing_operation" , although the previous message contained a value of
    total_tasks that is greater than one, indicating that the current plan is not completed and thus that the robot has
    been stuck in that task for a time greater than the threshold.
    """

    def __init__(self):
        self.time_of_last_op = datetime.now()
        self.open_tasks = 0
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)
        rospy.Subscriber("/arox/ongoing_operation", arox_operational_param, self.operation_callback, queue_size=1)
        self.mission_check()

    def mission_check(self):
        """
        Performs the simple mission failure check:
        Plan not completed and idle time limit during active plan exceeded -> mission failed
        """
        time_since_last_operation = (datetime.now() - self.time_of_last_op).total_seconds()
        if time_since_last_operation > config.MISSION_IDLE_LIMIT and self.open_tasks > 0:
            self.catastrophe_pub.publish(config.MISSION_FAIL_MSG)

    def operation_callback(self, msg):
        """
        Callback that receives information about the progress of the mission.

        @param msg: callback message containing mission info
        """
        self.open_tasks = msg.total_tasks
        self.time_of_last_op = datetime.now()


def node():
    """
    Simple mission monitoring node.
    """
    rospy.init_node('mission_monitoring')
    rospy.loginfo("launch LTA mission monitoring node..")
    MissionMonitor()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
