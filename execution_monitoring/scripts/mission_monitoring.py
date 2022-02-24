#!/usr/bin/env python

import rospy
from arox_performance_parameters.msg import arox_operational_param
from datetime import datetime
from execution_monitoring import config
from std_msgs.msg import String


class MissionMonitor:

    def __init__(self):
        rospy.Subscriber('/arox/ongoing_operation', self.operation_callback, arox_operational_param, queue_size=1)
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)

        self.mission_check()
        self.time_since_last_op = None

    def mission_check(self):
        # plan not completed and idle time limit during active plan exceeded -> mission failed
        if (datetime.now() - self.time_since_last_op).total_seconds() > config.MISSION_IDLE_LIMIT and self.open_tasks > 0:
            self.catastrophe_pub.publish(config.MISSION_FAIL_MSG)

    def operation_callback(self, msg):
        self.open_tasks = msg.total_tasks
        self.time_since_last_op = datetime.now()

def node():
    rospy.init_node('mission_monitoring')
    rospy.loginfo("launch LTA mission monitoring node..")
    MissionMonitor()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
