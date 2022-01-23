#!/usr/bin/env python
import rospy
from execution_monitoring import config
from std_msgs.msg import String, UInt16
from arox_performance_parameters.msg import arox_operational_param
from datetime import datetime


class PlanDeploymentMonitor:

    def __init__(self):
        rospy.Subscriber('arox/ongoing_operation', arox_operational_param, self.operation_callback, queue_size=1)
        rospy.Subscriber('/plan_retrieval_failure', UInt16, self.plan_fail_callback, queue_size=1)

        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)

        self.last_op_time = datetime.now()
        self.start_monitoring()

    def start_monitoring(self):
        while not rospy.is_shutdown():
            time_since_last_op = (datetime.now() - self.last_op_time).total_seconds()
            if time_since_last_op > config.IDLE_THRESH:
                rospy.loginfo("time since last op: %.2f s", round(time_since_last_op, 2))
                # extended idle time -- worth an operator notification
                self.robot_info_pub.publish(config.PLAN_DEPLOYMENT_FAILURE_ONE)
            rospy.sleep(config.MON_FREQ)
    
    def operation_callback(self, operation_state):
        rospy.loginfo("state of operation: %s", operation_state)
        self.time_since_last_op = datetime.now()
    
    def plan_fail_callback(self, plan_fail_code):
        if plan_fail_code == 0:
            rospy.loginfo("plan retrieval service timeout: %s", plan_fail_code)
            self.contingency_pub.publish(config.PLAN_DEPLOYMENT_FAILURE_TWO)
        elif plan_fail_code == 1:
            rospy.loginfo("empty plan: %s", plan_fail_code)
            self.contingency_pub.publish(config.PLAN_DEPLOYMENT_FAILURE_THREE)
        elif plan_fail_code == 2:
            rospy.loginfo("corrupted / infeasible plan: %s", plan_fail_code)
            self.contingency_pub.publish(config.PLAN_DEPLOYMENT_FAILURE_FOUR)
        else:
            rospy.loginfo("unknown plan fail code: %s", plan_fail_code)
            self.contingency_pub.publish(config.PLAN_DEPLOYMENT_FAILURE_FIVE)


def node():
    rospy.init_node('plan_deployment_monitoring')
    PlanDeploymentMonitor()
    rospy.loginfo("launch plan deployment monitoring..")
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
