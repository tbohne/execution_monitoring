#!/usr/bin/env python
import rospy
from execution_monitoring import config
from std_msgs.msg import String
from arox_performance_parameters.msg import arox_operational_param


class PlanDeploymentMonitor:

    def __init__(self):
        rospy.Subscriber('arox/ongoing_operation', arox_operational_param, self.operation_callback, queue_size=1)
        rospy.Subscriber('/plan_retrieval_exception', String, self.idle_exception_callback, queue_size=1)
    
    def operation_callback(self, operation_state):
        rospy.loginfo("STATE OF OP: %s", operation_state)
    
    def idle_exception_callback(self, msg):
        rospy.loginfo("plan retrieval exception: %s", msg)


def node():
    rospy.init_node('plan_deployment_monitor')
    PlanDeploymentMonitor()
    rospy.loginfo("launch plan deployment monitoring..")
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
